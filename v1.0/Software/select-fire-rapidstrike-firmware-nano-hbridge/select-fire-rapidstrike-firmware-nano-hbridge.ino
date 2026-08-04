/* ------------------------------------------------------------------------

Select-Fire Rapidstrike Kit firmware - Arduino Nano build for an external
DRV8871-based H-Bridge motor driver module (e.g. Adafruit's DRV8871 DC Motor
Driver Breakout, product #3190), instead of the v1.0 board's onboard
half-bridge FETs (see v1.0/Software/select-fire-rapidstrike-firmware/ for
that version). See this directory's README.md for the pin map, wiring
notes, and the debugging history behind the design choices below.

LOW_SIDE_PIN (D10) and HIGH_SIDE_PIN (D11) drive the DRV8871's IN1 and IN2
respectively - NOT what the macro names suggest; they're legacy from the
original half-bridge design (see the comment above their #defines below).
Unlike the onboard v1.0 board's discrete P-channel-high/N-channel-low FET
pair, the DRV8871 is a simple two-input full H-Bridge: (0,0)=coast,
(1,0)/(0,1)=drive, (1,1)=brake (both low-side FETs shorted - real active
braking, not just cutting power). Per the DRV8871 datasheet's recommended
PWM technique (TI SLVSCY9B, "Bridge Control"), one input is held fixed HIGH
while driving or braking and the other does the actual PWM switching
between LOW (driving) and HIGH (braking) - see driveDrv8871PWM(). The
datasheet's own worked example fixes IN1 and PWMs IN2, but motor direction
is irrelevant for this cam-driven pusher, so the roles are swapped here:
IN2 (HIGH_SIDE_PIN/D11) is the one held fixed, and IN1 (LOW_SIDE_PIN/D10)
is PWM'd, specifically so the PWM can use Timer1's flexible, exactly-tunable
frequency instead of Timer2's fixed-step frequency (D11's timer). This
firmware previously targeted a single low-side MOSFET switch module
(ProtoSupplies MOD-117) with no braking capability at all, which needed a
reduced PWM duty cycle just to keep the pusher's coasting momentum from
carrying it through extra, mis-firing cycles after being commanded to stop
- see README.md for that history. The DRV8871's real active braking should
make that speed reduction unnecessary; re-test at a higher
PUSHER_MOTOR_PWM_DUTY_PERCENT now that braking can actually arrest the
motor's momentum instead of merely letting it coast down.

Key differences from a naive "just wire it up" port, each hard-won from
actual hardware debugging (see README.md for the full story):
  - Cycle-control switch debounce cut to 2ms (from the trigger's 50ms):
    the switch is cam-actuated, not finger-actuated, and its contact
    closure duration shrinks as motor RPM increases - a 50ms debounce
    silently discards closures shorter than that as "noise," which meant
    at high pack voltage no cycle-control event ever registered at all,
    and nothing ever told the motor to stop.
  - Darts are counted on the cycle-control switch's RELEASE edge (pusher
    leaving retracted, i.e. the dart actually firing) rather than its
    PRESS edge (pusher returning home) - more immediate, and avoids
    hiding the same kind of debounce/timing problem above. The actual
    motor stop is still only ever committed once the pusher is confirmed
    back at retracted, so it never parks mid-stroke.
  - dartInProgress guards against a stray cycle-control PRESS event (e.g.
    leftover motion completing from before the current trigger pull, if
    the pusher hadn't fully settled at retracted yet) being mistaken for
    "cycle complete, fire again" - this caused a real extra shot.
  - handlePusherMotorFiring() re-asserts targetPusherState=ON on every
    single call, as a continuous LEVEL check, whenever firing is active
    and the cycle-control switch reads open - not just once, edge-
    triggered. At a higher PUSHER_MOTOR_PWM_DUTY_PERCENT (faster pusher),
    the previous edge-only approach let the pusher intermittently stall
    mid-stroke (never reaching retracted) if a brake/off command ever
    landed while the switch was still open, since nothing continuously
    re-checked afterward. Braking is still only ever committed on the
    switch's confirmed-closed edge (see wasCycCtrlSwPressed below) - this
    just guarantees driving can never be left stopped while open.
  - transitionHighSideFET()/setFETsForBraking() are written for the
    DRV8871's plain active-high IN1/IN2 logic and its (1,1)=brake
    convention - do NOT reuse these against the old MOD-117-style single
    low-side switch (or the onboard v1.0 half-bridge, which needs an
    inverted high-side signal and a different brake pattern) without
    re-deriving the correct truth table for that hardware first. Wiring
    the wrong logic to an H-Bridge can drive the motor continuously when
    the firmware thinks it's idle - verify this against your driver's
    actual datasheet before connecting it.
  - Braking (setFETsForBraking()) drives LOW_SIDE_PIN (IN1) fully high via
    transitionLowSideFET(), not through the PWM-duty path
    (driveDrv8871PWM()) used only for the actual "on"/firing state -
    PWM'ing what's supposed to be a solid brake signal would leave IN1 low
    part of the time, which isn't brake, it's an intermittent unwanted
    drive pulse every cycle.

Requires: JC_Button, CircularBuffer, arduino-timer (Arduino Library Manager).

------------------------------------------------------------------------ */

#include <Arduino.h>
#include <arduino-timer.h>
#include <JC_Button.h>
#include <CircularBuffer.hpp>

// Macros and vars for trigger and cycle control switch stuff
#define TRIGGER_PIN 2			// D2
#define CYC_CTRL_PIN 4			// D4

// Macros for FET pins. Physical wiring (confirmed against the actual board,
// NOT what the names suggest): LOW_SIDE_PIN(D10)=DRV8871 IN1,
// HIGH_SIDE_PIN(D11)=DRV8871 IN2. These names are legacy from the original
// half-bridge design and don't correspond to "high side"/"low side" FETs on
// this H-Bridge at all - don't infer IN1/IN2 from the macro names, always
// check this comment (or the datasheet-derived comments in
// transitionLowSideFET()/transitionHighSideFET()/driveDrv8871PWM() below).
//
// LOW_SIDE_PIN/D10 is the one that's actually PWM'd (via Timer1/OC1B),
// while HIGH_SIDE_PIN/D11 is held fixed via a plain digitalWrite() - the
// reverse of the DRV8871 datasheet's own worked example (which fixes IN1
// and PWMs IN2), but electrically equivalent since motor direction doesn't
// matter for this cam-driven pusher. D10 was chosen as the PWM'd pin so it
// could use Timer1 (16-bit, arbitrary frequency via ICR1) instead of
// Timer2 (8-bit, fixed-step frequencies only) - see "Why D10, not D11" in
// README.md.
//
// D10, not D5: D5 (Timer0) would have worked for a plain digitalWrite(),
// but Timer0 also drives millis()/micros(), which arduino-timer and
// JC_Button's debounce both depend on - reconfiguring Timer0 for a real PWM
// frequency would have corrupted that.
#define HIGH_SIDE_PIN 11			// D11 = DRV8871 IN2 - held HIGH while driving or braking, LOW only when fully off
#define LOW_SIDE_PIN 10				// D10 = DRV8871 IN1 - PWM'd, see driveDrv8871PWM()

// Macros to improve readability of FET transition functions
#define ON true
#define OFF false

// Macros for pusher drive state
#define PUSHER_DRIVE_STATE_OFF 0
#define PUSHER_DRIVE_STATE_BRAKE 1
#define PUSHER_DRIVE_STATE_ON 2

// Macros for current sensing
#define CURRENT_SENSE_PIN A1		// Nano A1
#define SENSE_RESISTANCE 0.01
// The Nano's ADC reference is its 5V supply.
#define BOARD_SUPPLY_VOLTAGE 5.0
#define NUM_OF_SAMPLES_FOR_DIFFERENTIATION 3

// Macros and vars for variable fire rate control values
#define POT_PIN A0			// Nano A0

// Macros for fire values
#define SAFETY 0
#define SEMI_AUTO 1
#define BURST_FIRE 2
#define FULL_AUTO 3

#define BURST_FIRE_LENGTH 3

// Macros to keep track of fire mechanisms
// Improve readability of pusher mechanism logic
#define PUSHER_MOTOR_FIRING_MECHANISM 0
#define SOLENOID_FIRING_MECHANISM 1

// Pin that is used to select pusher mechanism. Jumper off for motor, jumper on
// for solenoid. Uses internal pullup so HIGH = motor, LOW = solenoid
#define PUSHER_MECHANISM_SELECT_PIN 12		// D12

// PWM frequency (Hz) used to drive the pusher motor via IN1 (LOW_SIDE_PIN/
// D10) - see driveDrv8871PWM(). D10/Timer1 is a 16-bit timer with a freely
// adjustable ICR1-as-TOP, so (unlike Timer2/D11, only selectable in fixed
// prescaler steps) this hits the target frequency exactly. Default is
// 15kHz: comfortably within the DRV8871's 0-100kHz PWM rating, and high
// enough to stay above the range where PWM'd brushed motors tend to
// produce audible whine/cogging at lower frequencies. This is a
// compile-time constant, not pot-controlled like rateOfFire, because the
// right value depends on the specific motor and battery voltage in a given
// build, not something to tune in real time:
//   - Motor choice: smaller/lower-inductance motors respond more readily to
//     each individual pulse (more audible buzz, rougher low-speed
//     behavior) and may want a higher pulse rate for smoother operation;
//     larger/higher-inductance motors may tolerate a lower rate fine.
//   - Battery voltage: higher pack voltage delivers more energy per pulse
//     at a given duty cycle/frequency, so builds running higher voltage
//     packs may want a different pulse rate to keep motor behavior
//     (smoothness, heat, torque ripple) similar to what it was tuned for
//     at a lower voltage.
// Raise or lower this and re-test if a specific motor/battery combination
// sounds rough, runs hot, or doesn't spin smoothly at low commanded speeds.
#define PUSHER_MOTOR_PULSE_RATE 15000

// Timer1 Fast PWM (mode 14, ICR1 as TOP) register value derived from
// PUSHER_MOTOR_PULSE_RATE, prescaler=1. TOP = F_CPU/(prescaler*freq) - 1.
// Recalculates automatically if PUSHER_MOTOR_PULSE_RATE changes.
#define PUSHER_MOTOR_PWM_TOP ((F_CPU / (1UL * PUSHER_MOTOR_PULSE_RATE)) - 1)

// Fraction of each PWM period (percent) spent actually driving the motor,
// per the DRV8871 datasheet's recommended technique (7.3.1 "Bridge
// Control"): hold one input fixed HIGH (HIGH_SIDE_PIN/IN2/D11, see
// transitionHighSideFET()), and alternate the other (LOW_SIDE_PIN/IN1/D10)
// between LOW (driving) and HIGH (braking/slow decay) - see
// driveDrv8871PWM(). Motor direction is irrelevant for this cam-driven
// pusher, so either DRV8871 input could be the fixed one vs. the PWM'd
// one; IN1/D10 was chosen as the PWM'd pin specifically so it could use
// Timer1's flexible, exactly-tunable frequency. Tuned empirically on the
// equivalent MOD-117-based build's motor: high enough for reasonable
// speed, low enough that coasting momentum after a commanded stop can't
// carry the pusher through another full (mis-firing) cycle. Field-tested
// on this DRV8871 build starting from 50% (the datasheet's suggested
// starting point), then 70%, both confirmed working on actual hardware
// (see README.md's "Ensuring full retraction" and "Serial baud rate"
// sections for two real bugs 70% surfaced and their fixes - neither was
// actually caused by the duty cycle itself). Raise further and re-test if
// even more speed is wanted; watch for the extra-shot symptom described in
// README.md's "pusher-runaway investigation" section if coasting starts
// overshooting again.
// TODO: derive this from POT_PIN instead of a fixed value.
#define PUSHER_MOTOR_PWM_DUTY_PERCENT 90

// OCR1B compare value that produces PUSHER_MOTOR_PWM_DUTY_PERCENT worth of
// "driving" (IN1 LOW) time - see driveDrv8871PWM(). Timer1's non-inverting
// Fast PWM makes OC1B HIGH for a fraction of the period equal to
// (OCR1B+1)/(TOP+1), and here HIGH means "braking" (IN1=1), not "driving" -
// so this is deliberately the INVERSE of the duty percent: 0% drive ->
// OCR1B=TOP+1 (always braking, via the "set above TOP" trick - see
// transitionLowSideFET()), 100% drive -> OCR1B=0 (always driving).
#define PUSHER_MOTOR_PWM_BRAKE_LEVEL \
	((unsigned long)(PUSHER_MOTOR_PWM_TOP + 1) - (((unsigned long)(PUSHER_MOTOR_PWM_TOP + 1) * PUSHER_MOTOR_PWM_DUTY_PERCENT) / 100))

// Pin used to enable/disable the firmware's own software current sensing/OCP.
// This is about whether your motor-drive hardware exposes an analog
// current-sense signal for this pin to read, NOT about whether you're using
// an H-Bridge - plenty of H-Bridge driver modules (e.g. the DRV8871) handle
// overcurrent protection entirely internally with no exposed sense signal,
// in which case this should stay unjumpered (disabled) same as a simple
// low-side switch module with no current-sense output. Jumper on (LOW) only
// if CURRENT_SENSE_PIN is actually wired to a real current-sense signal -
// enabling this with CURRENT_SENSE_PIN floating will falsely trip OCP from
// ADC noise.
#define CURRENT_SENSE_SELECT_PIN 3 // D3

// Macros and vars for fire mode select pins. All active-low with internal
// pullups. ROT_SW_SAFETY_PIN always overrides the other three when grounded.
// Below that, only ROT_SW_SEMI_AUTO_PIN and ROT_SW_BURST_FIRE_PIN need to be
// wired for a basic 3-mode (semi/burst/full) build: full auto is the default
// when neither is grounded, so it needs no wire of its own. Wiring
// ROT_SW_FULL_AUTO_PIN low is optional and only meaningful for builds that
// wire all 3 non-safety positions explicitly - it produces the same result
// (FULL_AUTO) as the default case. See setCurrentFiremode().
#define ROT_SW_SAFETY_PIN 6		// D6
#define ROT_SW_SEMI_AUTO_PIN 7		// D7
#define ROT_SW_BURST_FIRE_PIN 8	// D8
#define ROT_SW_FULL_AUTO_PIN 9		// D9

// Array of all rotary switch pins, used only to pinMode() them all as
// INPUT_PULLUP in initRotSwPins() - fire mode itself is decided by
// setCurrentFiremode() reading these pins directly, not by scanning this array.
const uint8_t ROT_SW_PINS[] = {ROT_SW_SAFETY_PIN, ROT_SW_SEMI_AUTO_PIN, ROT_SW_BURST_FIRE_PIN, ROT_SW_FULL_AUTO_PIN};

// Function prototypes incase you're not using Arduino IDE
// I'd highly recommend using Platform.io instead of Arduino IDE
void initSerial(void);
void initFETPins(void);

void controlMotors();
bool handleComplementaryFETTransition(void *);
void turnOffAllFETs();
void setFETsForBraking();
void setFETsForOn();
void transitionHighSideFET(bool);
void transitionLowSideFET(bool);
void driveDrv8871PWM();

void handleCycleControl();

void handleFiring();

void setCurrentFiremode();
void printCurrentFireMode();
void setPusherMechanism();
void setRateOfFire();
void setDartsToFire();

void handlePusherMotorFiring();
void updatePusherMotorTiming();
bool turnOnPusherMotor(void *);
void handlePusherMotorSemiAutoAndBurst();
void handlePusherMotorFullAuto();

void handleSolenoidFiring();
void updateSolenoidTiming();
bool handleSolenoidTuronOn(void *);
void handleSolenoidSemiAutoAndBurst();
void handleSolenoidFullAuto();

float analogReadingToVoltage();
float voltageToCurrent(float);
void updateCurrentSenseValues(float);

Button trgSw (TRIGGER_PIN, 50, true, true);
// Debounce time cut way down from the default/trigger's 50ms: the cycle-
// control switch is cam-actuated, not finger-actuated, and its contact
// closure duration shrinks as motor RPM increases. At high pack voltage the
// closure can become shorter than a 50ms debounce window, which causes
// JC_Button's re-check-after-delay algorithm to discard every closure as
// "just noise" - see read()'s DEBOUNCE case. That was the actual cause of
// a pusher runaway seen during development: no cycle-control event ever
// registered at high speed, so nothing ever told controlMotors() to stop.
//
// invert=true, matching trgSw: the switch is wired closed (LOW, via
// INPUT_PULLUP) when the pusher is fully retracted, open (HIGH) partway
// through the stroke. With invert=true, isPressed()==true means "at
// rest/retracted", so wasPressed() fires on the HIGH->LOW transition - the
// pusher arriving back at retracted, i.e. cycle complete.
Button cycCtrlSw (CYC_CTRL_PIN, 2, true, true);

// variables for managing all of firing state
struct firingState {
	uint8_t currentPusherState = PUSHER_DRIVE_STATE_OFF;
	// should only be mutated in controlMotors(), but can be accessed anywhere
	uint8_t targetPusherState = PUSHER_DRIVE_STATE_OFF;

    // Switch states
    bool wasTriggerPulled = false;
    bool isTriggerPressed = false;

    bool isCycCtrlSwPressed = false;
    bool wasCycCtrlSwPressed = false;
    // Pusher just left the retracted position (a dart is being fired) -
    // see handlePusherMotorFiring() for why darts are counted on this edge
    // rather than on wasCycCtrlSwPressed.
    bool wasCycCtrlSwReleased = false;

    // Decided the moment a dart fires (wasCycCtrlSwReleased) - whether the
    // in-progress cycle should be the last one. Acted on only once the
    // pusher actually returns to retracted (wasCycCtrlSwPressed), so the
    // motor never stops mid-stroke.
    bool stopAfterThisCycle = false;

    // True from the moment a dart-fired (wasCycCtrlSwReleased) event is
    // counted until its matching retracted (wasCycCtrlSwPressed) event is
    // processed. Guards against a wasCycCtrlSwPressed event being treated
    // as "this cycle is done" when it's actually leftover motion completing
    // from BEFORE this trigger pull (e.g. the pusher hadn't fully settled
    // at retracted yet) - reacting to that stray event caused an extra shot,
    // since stopAfterThisCycle was still false at that point.
    bool dartInProgress = false;

	// Keep track of firing mechanism, solenoid or pusher. This is updated on
	// trigger pull by reading the jumper
	bool pusherMechanism = PUSHER_MOTOR_FIRING_MECHANISM;

	uint8_t currentFireMode = FULL_AUTO;

	// Keep track if pusher is firing. I can't just check if the pusher is being
	// powered because sometimes, the pusher is firing but off (for example, a
	// solenoid returning to its retracted position)
	bool isFiring = false;

	// Keeps track of how many darts to fire in semi- and burst-fire modes
	uint8_t dartsToFire = 0;

	// Keep track of how many darts have been fired
	uint8_t dartsFired = 0;

	// Rate of fire from 0 to 100 (100 fastest)
	uint8_t rateOfFire = 100;

	// Off time between shots when in pusher motor mode. Higher rate of fire means
	// higher off time between shots. Units in ms
	uint8_t pusherMotorOffTime = 0;

	// Duty cyle for solenoid firing. Different fire rates will be a factor of
	// this. Units are percent
	const uint8_t SOLENOID_DUTY_CYCLE = 60;

	// Time that solenoid is powered when firing, in ms
	uint16_t solenoidOnTime = 35;

	// Time that solenoid is off when firing, in ms
	uint16_t solenoidOffTime = 15;

	// Timer to keep track of when to turn on/off the pusher
	Timer<> firingTimer = timer_create_default();

} firingState;

// variables for managing all of MOSFET state
// these values should only be mutated in a FET function
// I should probably abstract this into its own class
struct fetState {
	// Timer for managing shoot through delay.
	// From this lib: https://github.com/contrem/arduino-timer
	// Quick description: Non-blocking library for delaying function calls
	Timer<> shootThroughProtectionTimer = timer_create_default();

	// Shoot through delay, in ms.
	// This value is read-only, so no need to store it in dynamic memory
	const uint8_t SHOOT_THROUGH_DELAY = 1;

	// Flag to indicate that a complementary FET transition has begun
	// This is mainly so multiple shoot-through timers doesn't start
	bool hasComplementaryTransitionBegun = false;

	bool currentHighSideFETState = OFF;
	bool targetHighSideFETState = OFF;

	bool currentLowSideFETState = OFF;
	bool targetLowSideFETState = OFF;

} fetState;

// Struct to keep track of sense resitors values for differentiation
struct currentSenseState {
	const uint8_t SAMPLE_DELAY = 15;	// Current sample delay, in ms

	// Current limit for OCP
	// If this limit is hit for longer than MAX_OCP_SAMPLES samples, then load
	// will be turned off
	// This is only for flywheel operation
	const uint8_t OCP_CURRENT_LIMIT = 5;

	// If more than OCP_CURRENT_LIMIT is observed for MAX_OCP_SAMPLES samples,
	// power to the load will be cut
	const uint8_t MAX_OCP_SAMPLES = 25;

	// Flag if OCP Threshold counter hit limit
	// flag to see if too much current. If this flag is true, load will turn off
	// in controlMotors()
	bool isTooMuchCurrent = false;

	// Counter to keep track how many times current is above OCP_CURRENT_LIMIT
	// Gets reset to 0 if current is less than OCP_CURRENT_LIMIT
	uint8_t ocpThresholdHitCounter = 0;

	// const uint8_t NUM_OF_SAMPLES_FOR_DIFFERENTIATION = 5;

	// Paralleling FIFO buffer for sample time (x) and current (y)
	CircularBuffer<float, NUM_OF_SAMPLES_FOR_DIFFERENTIATION> sampleTimeBuffer;
	CircularBuffer<float, NUM_OF_SAMPLES_FOR_DIFFERENTIATION> currentValues;

	Timer<> currentSenseTimer = timer_create_default();

	// -1 instantaneousChangeInCurrent means that this value isn't ready yet
	// instantaneousChangeInCurrent might not be ready if there aren't enough
	// samples
	float instantaneousChangeInCurrent = -1;

	float instantaneousCurrentDraw = -1;
} currentSenseState;

void setup() {
	initSerial();
	initFETPins();
	initRotSwPins();

	pinMode(PUSHER_MECHANISM_SELECT_PIN, INPUT_PULLUP);
	pinMode(CURRENT_SENSE_SELECT_PIN, INPUT_PULLUP);

	trgSw.begin();
	cycCtrlSw.begin();

	// Sets current sensing based on reading jumper pin (D3). If high (no
	// jumper cap on), current sensing is disabled - this is the correct
	// setting whenever CURRENT_SENSE_PIN isn't wired to a real signal,
	// regardless of whether your motor driver is an H-Bridge or not. If low
	// (jumper cap on), current sensing is enabled - only do this if
	// CURRENT_SENSE_PIN is actually connected to a real current-sense output.
	if (!digitalRead(CURRENT_SENSE_SELECT_PIN)) {
		// Start timer that executes monitorCurrent() every
		// currentSenseState.SAMPLE_DELAY ms
		currentSenseState.currentSenseTimer
			.every(currentSenseState.SAMPLE_DELAY, monitorCurrent);
	}

}

void loop() {
	fetState.shootThroughProtectionTimer.tick();
	currentSenseState.currentSenseTimer.tick();
	firingState.firingTimer.tick();

	handleTriggerPull();
	handleCycleControl();
	handleFiring();
 	controlMotors();


}

void initSerial() {
	// 115200, not 9600: this firmware prints substantial diagnostics (dart-
	// fired/retracted edges, mode/ROF lines) right at the events that also
	// drive FET-transition timing. At 9600 baud a single ~50-byte line takes
	// ~50ms to transmit; once the AVR's 64-byte TX buffer fills faster than
	// it drains, Serial.print() busy-waits for space, stalling loop() -
	// including fetState.shootThroughProtectionTimer.tick(), which is what
	// actually re-engages the motor after the brief commanded OFF/coast
	// between BRAKE and the next ON. That turned a designed ~1ms shoot-
	// through gap into however long the print happened to block for,
	// producing a real, physically-felt stutter with cycle-to-cycle
	// cadence that looked alternating/inconsistent in the logs - not a
	// motor or H-Bridge problem, a print-speed problem. 115200 makes each
	// line's transmit time negligible against the timers it was delaying.
	Serial.begin(115200);
	Serial.println("Serial communication established");
}

void initFETPins() {
	pinMode(HIGH_SIDE_PIN, OUTPUT);
	pinMode(LOW_SIDE_PIN, OUTPUT);

	// Configure Timer1 for Fast PWM (mode 14, ICR1 as TOP) on OC1B (D10) at
	// PUSHER_MOTOR_PULSE_RATE Hz, non-inverting, prescaler=1. Only OC1B's
	// compare output is enabled (COM1B1) - OC1A/D9 is deliberately left as
	// plain digital I/O (ROT_SW_FULL_AUTO_PIN is D9 and must stay a normal
	// input). Timer1 is independent of Timer0, so this doesn't touch
	// millis()/micros() at all. Starts at 0% duty (solid LOW = coast) until
	// commanded on.
	TCCR1A = (1 << COM1B1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
	ICR1 = PUSHER_MOTOR_PWM_TOP;
	OCR1B = 0;

	firingState.targetPusherState = PUSHER_DRIVE_STATE_OFF;

	controlMotors();
}

void initRotSwPins() {
  // Set all rotary switch pins to inputs with pullups
  for (int i = 0; i < sizeof(ROT_SW_PINS)/sizeof(ROT_SW_PINS[0]); i++) {
    pinMode(ROT_SW_PINS[i], INPUT_PULLUP);
  }
}

// Directly handles driving the pusher, including shoot-through protection
// Controls motors based on firingState currentPusherState and targetPusherState
void controlMotors() {
	// If too much current, cut off power to load and don't run anything else
	if (currentSenseState.isTooMuchCurrent) {
		Serial.println("OCP");

		turnOffAllFETs();

		firingState.targetPusherState = PUSHER_DRIVE_STATE_OFF;
    firingState.isFiring = false;

		return;
	}

	if (firingState.targetPusherState == PUSHER_DRIVE_STATE_OFF) {
        turnOffAllFETs();

        firingState.currentPusherState = PUSHER_DRIVE_STATE_OFF;

        // Targetstate requires MOSFETs to change state that's at risk of shoot-
        // through
        // When transition FETs where the FETs are complementary, here's the sequence:
        // BRAKE -> OFF -> [shoot-through delay] -> ON
        // (or)
        // ON -> OFF -> [shoot-through delay] -> BRAKE
  		// Make sure pusher state changed
    } else if (firingState.currentPusherState != firingState.targetPusherState
  	&& !fetState.hasComplementaryTransitionBegun
  	// Make sure target pusher state results in a complementary FET state
  	&& (firingState.targetPusherState == PUSHER_DRIVE_STATE_ON
  	|| firingState.targetPusherState == PUSHER_DRIVE_STATE_BRAKE)) {

   	    turnOffAllFETs();

   	    // Set flag to indicate that transition of FETs to a complementary state has
   	    // begun
   	    fetState.hasComplementaryTransitionBegun = true;

   	    // Transition FETs to match their target state afte SHOOT_THROUGH_DELAY
   	    fetState.shootThroughProtectionTimer
   		    .in(fetState.SHOOT_THROUGH_DELAY, handleComplementaryFETTransition);
  }
}

// Actually drives FETs in complementary switching
bool handleComplementaryFETTransition(void *) {
	if (firingState.targetPusherState == PUSHER_DRIVE_STATE_ON) {
		setFETsForOn();
	} else if (firingState.targetPusherState == PUSHER_DRIVE_STATE_BRAKE) {
		setFETsForBraking();
	}

	// Reset flag, complementary transition complete
 	fetState.hasComplementaryTransitionBegun = false;

 	// Target pusher state has been achieved
	firingState.currentPusherState = firingState.targetPusherState;

	return true;		//required by timer lib
}

// IN1/IN2 truth table for this DRV8871-based H-Bridge: (0,0)=coast,
// (1,0)/(0,1)=drive (direction is arbitrary/symmetric for a pusher - swap
// the two motor leads on the terminal block if it spins the wrong way),
// (1,1)=brake (both low-side FETs on, shorting the motor for real active
// braking). LOW_SIDE_PIN(D10)=IN1, HIGH_SIDE_PIN(D11)=IN2. Per the
// datasheet's recommended PWM technique (TI SLVSCY9B, "Bridge Control"),
// one input is held fixed while the other does the actual switching; since
// direction doesn't matter for this cam-driven pusher, the roles are
// swapped from the datasheet's own example here: HIGH_SIDE_PIN/IN2 is the
// fixed plain digitalWrite() pin, and LOW_SIDE_PIN/IN1 is the PWM'd pin
// (see driveDrv8871PWM()) - chosen so the PWM lands on Timer1 (D10),
// which can hit an exact target frequency, rather than Timer2 (D11).
void turnOffAllFETs() {
	transitionLowSideFET(OFF);
	transitionHighSideFET(OFF);
}

void setFETsForBraking() {
	transitionLowSideFET(ON);
	transitionHighSideFET(ON);
}

void setFETsForOn() {
	transitionHighSideFET(ON);
	driveDrv8871PWM();
}

// Drives LOW_SIDE_PIN (D10/IN1) fully on (solid, not PWM'd) or off, via
// Timer1/OC1B. Used for braking and for turning off - braking specifically
// needs a solid, continuous HIGH on IN1 (with IN2 already held HIGH by
// transitionHighSideFET()) to short the motor terminals; PWM'ing this pin
// during "brake" would leave IN1 low part of the time, which isn't brake at
// all, it's a brief unwanted drive pulse every cycle. See driveDrv8871PWM()
// for the actual speed-controlled "on" state used while firing.
//
// Setting OCR1B above TOP (PUSHER_MOTOR_PWM_TOP+1) is the standard trick
// for a solid, continuously-high output in this PWM mode: the compare
// match that would pull the pin low never occurs within the counting range.
void transitionLowSideFET(bool toTurnOn) {
	OCR1B = toTurnOn ? (PUSHER_MOTOR_PWM_TOP + 1) : 0;
}

// Abstracts away digitalWrite() on HIGH_SIDE_PIN (D11/IN2). Plain
// active-high logic - DRV8871's IN2 is a normal logic input, not an
// inverted P-channel gate the way the original discrete-FET half-bridge's
// high-side pin was. Held HIGH for both "on" and "brake" (IN2 stays fixed
// per the datasheet's recommended PWM technique - only IN1 toggles), and
// LOW only when fully off (coast).
void transitionHighSideFET(bool toTurnOn) {
	digitalWrite(HIGH_SIDE_PIN, toTurnOn);
}

// Drives the pusher motor at PUSHER_MOTOR_PWM_DUTY_PERCENT duty cycle by
// PWM'ing LOW_SIDE_PIN (D10/IN1) between LOW (driving) and HIGH (braking/
// slow decay) via Timer1/OC1B, per the DRV8871 datasheet's recommended PWM
// technique (with IN1/IN2 roles swapped from the datasheet's own example,
// since direction doesn't matter here) - IN2 (HIGH_SIDE_PIN) stays fixed
// HIGH throughout, set by transitionHighSideFET() in setFETsForOn(). Used
// only while actually firing (setFETsForOn()), never for braking (see
// transitionLowSideFET()).
void driveDrv8871PWM() {
	OCR1B = PUSHER_MOTOR_PWM_BRAKE_LEVEL;
}

void handleTriggerPull() {
  trgSw.read();

  firingState.isTriggerPressed = trgSw.isPressed();
  firingState.wasTriggerPulled = trgSw.wasPressed();

  // Prints the instant the debounced trigger reads released - useful when
  // diagnosing whether isTriggerPressed actually goes false promptly when
  // the trigger is physically let go.
  if (trgSw.wasReleased()) {
    Serial.println("TRIGGER RELEASED");
  }

  // Prints every NEW debounced press edge (not just "currently pressed") -
  // this is what re-arms the whole firing sequence from scratch (resets
  // dartsFired, re-picks fire mode, sets pusher back to ON). If this prints
  // more than once per actual physical trigger squeeze, something is
  // registering a spurious press edge, independent of what your finger is doing.
  if (firingState.wasTriggerPulled) {
    Serial.println("TRIGGER PRESSED (new edge)");
  }
}

void handleCycleControl() {
  cycCtrlSw.read();
  firingState.isCycCtrlSwPressed = cycCtrlSw.isPressed();
  firingState.wasCycCtrlSwPressed = cycCtrlSw.wasPressed();
  firingState.wasCycCtrlSwReleased = cycCtrlSw.wasReleased();

  // Prints every debounced cycle-control transition (both edges), with
  // running counts and timestamps - useful for confirming the actual rate
  // these fire at and correlating against dartsFired when diagnosing
  // firing-sequence issues.
  if (firingState.wasCycCtrlSwPressed) {
    static uint16_t cycCtrlPressedCount = 0;
    cycCtrlPressedCount++;
    Serial.print("CYC CTRL PRESSED (retracted) #"); Serial.print(cycCtrlPressedCount);
    Serial.print(" at t="); Serial.println(millis());
  }
  if (firingState.wasCycCtrlSwReleased) {
    static uint16_t cycCtrlReleasedCount = 0;
    cycCtrlReleasedCount++;
    Serial.print("CYC CTRL RELEASED (dart fired) #"); Serial.print(cycCtrlReleasedCount);
    Serial.print(" at t="); Serial.println(millis());
  }
}

void handleFiring() {
	if (firingState.wasTriggerPulled) {
    setCurrentFiremode();
		// printCurrentFireMode();

    setPusherMechanism();
    setRateOfFire();
    setDartsToFire();

    firingState.isFiring = true;

  }

  // If pusher in firing sequence
  if (firingState.isFiring) {
  	if (firingState.pusherMechanism == PUSHER_MOTOR_FIRING_MECHANISM) {
			handlePusherMotorFiring();
    } else if (firingState.pusherMechanism == SOLENOID_FIRING_MECHANISM) {
    	handleSolenoidFiring();
    }
  }
}

// Sets fire mode by reading the rotary switch pins directly, in priority
// order: SAFETY (ROT_SW_SAFETY_PIN) always overrides everything else. Below
// that, only two wired switches are needed to cover all 3 firing modes:
// ROT_SW_SEMI_AUTO_PIN low selects semi-auto, ROT_SW_BURST_FIRE_PIN low
// selects burst, and full auto is simply the default when neither of those
// is grounded - no third wire required. A build that wires all 3 positions
// explicitly (including ROT_SW_FULL_AUTO_PIN) still works identically, since
// grounding ROT_SW_FULL_AUTO_PIN also falls through to the same default.
void setCurrentFiremode() {
  if (digitalRead(ROT_SW_SAFETY_PIN) == LOW) {
    firingState.currentFireMode = SAFETY;
  } else if (digitalRead(ROT_SW_SEMI_AUTO_PIN) == LOW) {
    firingState.currentFireMode = SEMI_AUTO;
  } else if (digitalRead(ROT_SW_BURST_FIRE_PIN) == LOW) {
    firingState.currentFireMode = BURST_FIRE;
  } else {
    firingState.currentFireMode = FULL_AUTO;
  }
}

// Print fire modes over serial
void printCurrentFireMode() {
  if (firingState.currentFireMode == SAFETY) {
      Serial.println("Safety");
    } else if (firingState.currentFireMode == SEMI_AUTO) {
      Serial.println("Single Shot");
    } else if (firingState.currentFireMode == BURST_FIRE) {
      Serial.println("Burst Fire");
    } else if (firingState.currentFireMode == FULL_AUTO) {
      Serial.println("Full Auto");
    }
}

// Sets pusher mechanism based on reading jumper pin (D12)
// If high (no jumper cap on), pusher mech is pusher
// If low (jumper cap on), pusher mech is solenoid
void setPusherMechanism() {
	if (digitalRead(PUSHER_MECHANISM_SELECT_PIN)) {
		firingState.pusherMechanism = PUSHER_MOTOR_FIRING_MECHANISM;
	} else {
		firingState.pusherMechanism = SOLENOID_FIRING_MECHANISM;
	}
}

void setRateOfFire() {
    // These defaults presently apply only for PUSHER_MOTOR_FIRING_MECHANISM.
    // Upper limit capped well below the original 100: how ROF maps to real
    // pusher speed isn't well characterized, and the faster the motor spins,
    // the more the pusher's momentum can carry it past the retracted
    // position before braking actually arrests it - erring low here reduces
    // that overshoot/misfire risk. Raise cautiously, and only after
    // confirming the pusher reliably stops fully retracted at the new ceiling.
    uint8_t lowerLimitForRateOfFire = 1;
    uint8_t upperLimitForRateOfFire = 5;

	if (firingState.pusherMechanism == SOLENOID_FIRING_MECHANISM) {
		lowerLimitForRateOfFire = 20;
		upperLimitForRateOfFire = 35;
	}

	firingState.rateOfFire = map(
		analogRead(POT_PIN), 0, 1024,
		upperLimitForRateOfFire, lowerLimitForRateOfFire);

	Serial.println(firingState.rateOfFire);
}

void setDartsToFire() {
  if (firingState.currentFireMode == SEMI_AUTO) {
    firingState.dartsToFire = 1;
  } else if (firingState.currentFireMode == BURST_FIRE) {
    firingState.dartsToFire = BURST_FIRE_LENGTH;
  }

  firingState.dartsFired = 0;
  firingState.stopAfterThisCycle = false;
  firingState.dartInProgress = false;
}

void handlePusherMotorFiring() {
	// Continuous safety net, re-checked every call (a LEVEL check, not an
	// edge reaction): for as long as we're actively firing outside of
	// safety mode and the cycle-control switch reads open - i.e. the
	// pusher hasn't reached retracted yet - force the motor back to ON.
	// One dart-firing cycle is "driven until the switch opens, then
	// closes again" - braking may only ever be committed once the switch
	// is confirmed closed (see the wasCycCtrlSwPressed handling below),
	// and must never be left standing while the switch is still open. This
	// guards against the pusher ever stalling mid-stroke (short of fully
	// retracted) from any stray or premature brake/off command - the only
	// way to guarantee it reaches fully retracted is to keep driving for
	// as long as the switch is open. This is a no-op whenever
	// targetPusherState is already ON, so it never interferes with the
	// intentional brake-then-delayed-restart pause between shots below
	// (during which the switch reads closed the whole time).
	if (firingState.currentFireMode != SAFETY && !firingState.isCycCtrlSwPressed) {
		firingState.targetPusherState = PUSHER_DRIVE_STATE_ON;
	}

	// Start motors and firing sequence upon trigger pull when not in safety mode
	if (firingState.wasTriggerPulled) {
		updatePusherMotorTiming();

		if (firingState.currentFireMode != SAFETY) {
			firingState.targetPusherState = PUSHER_DRIVE_STATE_ON;

		// Firemode is safety
		} else {
			firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
			firingState.isFiring = false;
		}

	}

	// Pusher just left the retracted position - a dart is being fired right
	// now. This is a more immediate, more reliable signal than waiting for
	// the pusher to return home (wasCycCtrlSwPressed, below), and avoids
	// hiding switch debounce/timing problems the way that did during
	// development. Darts are counted here, and this is where we DECIDE
	// whether the in-progress cycle should be the last one - but we don't
	// act on that decision yet. The motor keeps running until the pusher
	// actually gets back to retracted, so it never stops mid-stroke
	// (misfire risk).
	if (firingState.wasCycCtrlSwReleased && firingState.currentFireMode != SAFETY) {
		firingState.dartsFired++;
		firingState.dartInProgress = true;

		if (firingState.currentFireMode == FULL_AUTO) {
			firingState.stopAfterThisCycle = !firingState.isTriggerPressed;
		} else {
			// SEMI_AUTO (dartsToFire=1) and BURST_FIRE (dartsToFire=3) both
			// stop once they've fired their configured number of darts.
			firingState.stopAfterThisCycle =
				(firingState.dartsFired >= firingState.dartsToFire);
		}

		Serial.print("[dartFired] mode="); Serial.print(firingState.currentFireMode);
		Serial.print(" trig="); Serial.print(firingState.isTriggerPressed);
		Serial.print(" darts="); Serial.print(firingState.dartsFired);
		Serial.print("/"); Serial.print(firingState.dartsToFire);
		Serial.print(" stopAfterThisCycle="); Serial.println(firingState.stopAfterThisCycle);
	}

	// Pusher has returned to fully retracted - the only point at which we
	// actually stop the motor, so it never ends up parked mid-stroke.
	if (firingState.wasCycCtrlSwPressed) {
		if (firingState.currentFireMode == SAFETY) {
			firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
			firingState.isFiring = false;
			return;
		}

		// If no dart-fired event has been counted since this trigger pull
		// (or since the last time we processed a retracted event), this
		// PRESSED event doesn't belong to a cycle we started - it's
		// leftover motion completing from before this sequence (e.g. the
		// pusher hadn't fully settled at retracted when the trigger was
		// pulled again). Ignore it: don't touch targetPusherState, don't
		// schedule anything. Reacting to this caused a genuine extra shot,
		// since stopAfterThisCycle was still false at that point.
		if (!firingState.dartInProgress) {
			Serial.println("CYC CTRL PRESSED ignored (no dart in progress)");
			return;
		}

		firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
		firingState.dartInProgress = false;

		if (firingState.stopAfterThisCycle) {
			firingState.dartsFired = 0;
			firingState.isFiring = false;
			firingState.stopAfterThisCycle = false;
			return;
		}

		// Still more darts to fire - power the pusher motor back on after a
		// slight delay proportional to rate of fire
		updatePusherMotorTiming();
		Serial.print("mode="); Serial.print(firingState.currentFireMode);
		Serial.print(" trig="); Serial.print(firingState.isTriggerPressed);
		Serial.print(" darts="); Serial.print(firingState.dartsFired);
		Serial.print("/"); Serial.print(firingState.dartsToFire);
		Serial.print(" ROF="); Serial.println(firingState.rateOfFire);
		firingState.firingTimer.in(
			firingState.pusherMotorOffTime, turnOnPusherMotor);
	}
}

// Update pusher motor off time based on pot reading
void updatePusherMotorTiming() {
	setRateOfFire();

	firingState.pusherMotorOffTime = firingState.rateOfFire * 2;

	// Serial.println(firingState.pusherMotorOffTime);
}

bool turnOnPusherMotor(void *) {
	if (firingState.currentFireMode != SAFETY) {
		// Don't fire if pusher is in full-auto and trigger isn't held
		if (firingState.currentFireMode == FULL_AUTO
			&& !firingState.isTriggerPressed) {
			return true;
		}

		firingState.targetPusherState = PUSHER_DRIVE_STATE_ON;
	}

	return true;
}

void handlePusherMotorSemiAutoAndBurst() {
  // If no more darts to fire, stop solenoid
  if (firingState.dartsFired >= firingState.dartsToFire) {
    firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
    firingState.dartsFired = 0;
    firingState.isFiring = false;

  // Still more darts to fire, so keep firing pusher
  } else {
  	firingState.targetPusherState = PUSHER_DRIVE_STATE_ON;

  }
}

void handlePusherMotorFullAuto() {
	// Trigger is let go and pusher is retracted, so turn off pusher
  if (!firingState.isTriggerPressed && firingState.isCycCtrlSwPressed) {
    firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
    firingState.isFiring = false;


  // Trigger still pressed, so keep firing pusher
  } else {
  	firingState.targetPusherState = PUSHER_DRIVE_STATE_ON;

  }
}

void handleSolenoidFiring() {
	// Right when trigger was pulled, initiate firing sequence
	if (firingState.wasTriggerPulled) {
		// Set solenoid on and off times based on rate of fire
		updateSolenoidTiming();

		// Turn solenoid on, set it to turn off later. Solenoid shouldn't be
		// turned on in safety
		if (firingState.currentFireMode != SAFETY) {
			firingState.targetPusherState = PUSHER_DRIVE_STATE_ON;
			firingState.firingTimer
				.in(firingState.solenoidOnTime, turnSolenoidOff);

		// Firemode is safety
		} else {
			firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
			firingState.isFiring = false;
		}
	}

}

// Sets solenoid timing based on rate of fire and duty cycle
void updateSolenoidTiming() {
	setRateOfFire();

	firingState.solenoidOnTime = (100.0 / firingState.rateOfFire)
		* (firingState.SOLENOID_DUTY_CYCLE / 2.0);

	firingState.solenoidOffTime = firingState.solenoidOnTime
		/ (firingState.SOLENOID_DUTY_CYCLE/(100.0/2.0));

	// Serial.print("On time "); Serial.println(firingState.solenoidOnTime);
	// Serial.print("Off time "); Serial.println(firingState.solenoidOffTime);
}

void handleSolenoidSemiAutoAndBurst() {
  // If no more darts to fire, stop solenoid
  if (firingState.dartsFired >= firingState.dartsToFire) {
    firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
    firingState.dartsFired = 0;
    firingState.isFiring = false;

  // Still more darts to fire, so keep firing pusher and continue firing loop
  } else {
  	turnSolenoidOn();
  }
}

void handleSolenoidFullAuto() {
	// Trigger is let go, so turn off solenoid
  if (!firingState.isTriggerPressed) {
    firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
    firingState.isFiring = false;

  // Trigger still pressed, so keep firing pusher and continue firing loop
  } else {
  	turnSolenoidOn();
  }
}

// Turns solenoid off and continues firing loop. Don't call this if you just
// want to turn off the solenoid
bool turnSolenoidOff(void *) {
	// Serial.println("Off");
	firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;

	firingState.firingTimer
		.in(firingState.solenoidOffTime, handleSolenoidTuronOn);


	return true;
}

// Called after solenoid turns off and about to turn back on. Ideally, this is
// executed when plunger in retraced position. Determines whether to continue
// firing the solenoid or to turn the solenoid off
bool handleSolenoidTuronOn(void *) {
	firingState.dartsFired++;

	// Update timing in case rate of fire changed
	updateSolenoidTiming();

	if (firingState.currentFireMode == SEMI_AUTO
		|| firingState.currentFireMode == BURST_FIRE) {
    handleSolenoidSemiAutoAndBurst();
  } else if (firingState.currentFireMode == FULL_AUTO) {
    handleSolenoidFullAuto();
  } else {
		firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
	}

	return true;
}

// Turns solenoid on and continues firing loop. Don't call this if you just
// want to turn on the solenoid
void turnSolenoidOn() {
	// Serial.println("On");
	firingState.targetPusherState = PUSHER_DRIVE_STATE_ON;

	firingState.firingTimer
 		.in(firingState.solenoidOffTime, turnSolenoidOff);
}

float analogReadingToVoltage() {
	return (analogRead(CURRENT_SENSE_PIN) * BOARD_SUPPLY_VOLTAGE)/1024.0;
}

float voltageToCurrent(float voltage) {
	return voltage/SENSE_RESISTANCE;	// Basic Ohm's law
}

bool monitorCurrent(void *) {
 	currentSenseState.instantaneousCurrentDraw = voltageToCurrent(analogReadingToVoltage());

	updateCurrentSenseValues(currentSenseState.instantaneousCurrentDraw);

	currentSenseState.instantaneousChangeInCurrent = differentiateCurrentSenseValues();

	// Increment ocpThresholdHitCounter if necessary
	if (currentSenseState.instantaneousCurrentDraw < currentSenseState.OCP_CURRENT_LIMIT) {
		resetOCPCounters();
	} else {
		currentSenseState.ocpThresholdHitCounter++;
	}

	// Set tooMuchCurrent flag if too much current
	if (currentSenseState.ocpThresholdHitCounter
		> currentSenseState.MAX_OCP_SAMPLES) {
		currentSenseState.isTooMuchCurrent = true;
	}


	// Serial.print("OCP counters: "); Serial.println(currentSenseState.ocpThresholdHitCounter);
	// Serial.print("I: "); Serial.println(currentSenseState.instantaneousCurrentDraw);


	// printCurrentSenseValues(currentSenseState.instantaneousCurrentDraw);


	// Must return tru because this function is executed through a timer function
	return true;
}

void updateCurrentSenseValues(float current) {
	// Update time and current arrays with new sample values
	currentSenseState.sampleTimeBuffer.push(millis()/1000.0);
	currentSenseState.currentValues.push(current);
}

float differentiateCurrentSenseValues() {
	// Only differentiate if current arr full
	if (currentSenseState.currentValues.isFull()
		&& currentSenseState.sampleTimeBuffer.isFull()) {

		float totaldI = 0;
		float totaldt = 0;

		// Calculations start at second value because the previous value is needed
		// to find slope. The first value doesn't have a previous value, so the
		// calculations won't work
		for (int i = 1; i < currentSenseState.currentValues.size(); i++) {
			totaldI += currentSenseState.currentValues[i]
				- currentSenseState.currentValues[i - 1];

			totaldt += currentSenseState.sampleTimeBuffer[i]
				- currentSenseState.sampleTimeBuffer[i - 1];

		}

		// printCurrentSenseDifferentiationBuffer();

		return totaldI/totaldt;


	}

	return -1;
}

void printCurrentSenseDifferentiationBuffer() {
	if (currentSenseState.currentValues.isFull()) {
		for (int i = 0; i < currentSenseState.currentValues.size(); i++) {
			Serial.print(i); Serial.print(": ");
			Serial.print(currentSenseState.sampleTimeBuffer[i]);
			Serial.print(", ");
			Serial.print(currentSenseState.currentValues[i]);
			Serial.println("A ");
		}
	} else {
		Serial.print("Not full yet! Here's the current length: ");
		Serial.println(currentSenseState.currentValues.size());
	}
}

void printCurrentSenseValues(float current) {
	Serial.print("I [A]: "); Serial.println(current);

	// Print instantaneous change in current if the value is valid
	if (currentSenseState.instantaneousChangeInCurrent > -1) {
		Serial.print("dI/dt [A/s]: ");
		Serial.println(currentSenseState.instantaneousChangeInCurrent);
	}

	// Print threshold counter
	// if (currentSenseState.OCP_CURRENT_LIMIT != 0) {
	// 	Serial.print("ocpThresholdHitCounter: ");
	// 	Serial.println(currentSenseState.ocpThresholdHitCounter);
	// }

	Serial.println("\n");
}

void resetOCPCounters() {
	currentSenseState.ocpThresholdHitCounter = 0;
	currentSenseState.isTooMuchCurrent = false;
}
