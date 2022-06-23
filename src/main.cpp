/*
To do:
--Add a flag for recording mode that I can access in the Setup Sensors menu page. Basically, I'm thinking that record mode will be either One Shot or Continuous.
    Continuous is for long periods of recording data, like logging pressure, temp, humidity, and UV while I climb a mountain. Want to see how those change over
    a long time, so they all go in one log file together. One shot isn't exactly a true one-shot, but I don't have a better term yet. One Shot is for periodic
    sensor readings, like the water sensors. Maybe call that mode Periodic. The idea is that I'll stop every 15 minutes or so and take a new set of samples
    in a lake or stream as I hike upstream. These short files will have many samples each, but I don't want to record continuously because the sensors won't be in
    the water while I hike, so I don't want to collect bad data. I could either figure out how to pause and resume recordings on a single file, or I can do what I'm proposing here:
    set a record mode flag to Periodic/One shot, and when I open a new file to log data to, it prepends the name of that file with "Periodic - " followed by the date,
    and then I know that any log files in that immediate date range that have that label are related to each other. I could also prepend Continuous mode files with 
    "Continuous - " so that I can later sort the log files more easily.
    --Make sure to include the "Record Mode" options in the setup Page. Make it a toggle too, maybe, so that clicking on the button switches some text (either on the
    button itself or beside it) between Continuous and Periodic. 
    -----actually going to call the modes STEADY and BURST. Shorter words will fit better on screen.
    --I could even add extra labels to the file to indicate which sensors are used, but I will also have those in the header of each CSV, so I'm not sure.
--Add automatic naming that includes the above labels and the date/time for new log files.
--consolidate GPS things into a function outside of main loop.
--Add a "Waiting for GPS Fix" master UI state. Right now the whole system waits to get a fix before proceeding to the main loop. Instead, I want to have that process
    delayed until I actually tell the system to start recording. Once I tell it to start recording, I want it to check for and wait for a GPS fix before switching to the 
    recording state and UI page. I'll need a subpage of the recording page that displays "Waiting for GPS fix" until a fix is established, and then switches to "recording active"
    when a fix is established.
    ----also add appropriate indicators to the lock screen to show if it's recording or not
--Break time display UI element into its own function, add time display to lock page and recording page
--Maybe even add a display element to the recording page that shows which sensors are being logged


Things I'd like to add:
    -last setup saved. like if the last time I logged data I enabled GPS, baro, UV index, and temperature, it would be nice to have those
    automatically enabled next time I boot up the system. Not sure how to save that kind of state yet.
        --Actually, I bet I can save those as a .ini file on the SD card that I could reference on reboot! Or maybe save it in PROGMEM? Not sure how that works. 
        Just looked, PROGMEM keyword puts things in flash rather than RAM. Seems perfect, I think. Look that up for the Feather Sense
    -ability to reset the RTC on the SD/RTC featherwing with the time from the GPS RTC. Could be an option on the setup page.

General notes:
    --I have a record mode flag now, which I can set to either recordingSteady or recordingBurst.
    --I think I might want to create a sensor class. Then each sensor could have an instantiation of the sensor class. Ideally, the class would store a bunch of info 
        for me, like the position of the UI buttons for that sensor, the Adafruit_GFX_Button object for the sensor, and the flag showing whether the sensor is enabled.
        I would also like to have an array or some structure holding all the sensor objects so that I could iterate through them. I think that might make it easier
        to do things like navigate the UI buttons. Not sure yet.
    --I know, I have two RTCs. However, the RTC on the SD/RTC module works with a convenient library, and can be reset to local time, which the GPS RTC can't,
        so using the SD RTC saves me a bunch of work. There's also an interrupt breakout on the SD RTC that could be useful. As far as setting RTC time, I just have
        it set by using the example sketch for the RTC that comes with the RTCLib. It sets the clock from the computer clock time at compile, so it's like 20 seconds 
        behind, but that's good enough. I have the battery backup installed, so I should be set on time for a long time. Note that the RTC doesn't account for DST,
        so I will eventually have to automate that myself somehow.
*/

/*
Reference info about the Feather Sense board, like built-in sensor addresses.
Adafruit reference list of i2c addresses: https://learn.adafruit.com/i2c-addresses/the-list

gyro/accelerometer          i2c: 0x6A, with an IRQ on digital pin 3
magnetometer                i2c: 0x1C
light/gesture/proximity     i2c: 0x39, with an IRQ on digital pin 36
humidity                    i2c: 0x44
temp/pressure               i2c: 0x77
pdm mic                     data on d34, clock on d35
RTC (on SD card; PCF8523)   i2c: 0x68
*/

/*
possible external sensors

particulate matter adafruit pmsa003l        i2c: 0x12
UV sensor                                   analog
Geiger counter!                             https://www.dfrobot.com/product-2547.html
total dissolved solids                      https://www.dfrobot.com/product-1662.html
turbidity                                   https://www.dfrobot.com/product-1394.html
water temp                                  https://www.dfrobot.com/product-1354.html
*/




#include <Arduino.h>
#include <Adafruit_GPS.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include <OneButton.h>              //handles navigation button events
#include <RTClib.h>


//SD card connections
#define MOSI 25     
#define MISO 24     
#define CLK 26      
#define CS 10       
//display connections
#define DISPLAY_CLK 5
#define DISPLAY_DI 6
#define DISPLAY_CS 9
//black and white definitions for display
#define BLACK 0
#define WHITE 1
//define the nav button pin connections
#define PIN_BUTTON_LEFT     19  //switching from pin 13 fixed my problems! Pin 13 shares the builtin led, so maybe using it as input was problem.
#define PIN_BUTTON_RIGHT    11
#define PIN_BUTTON_UP       12
#define PIN_BUTTON_DOWN     18     //for some reason down button stopped working on Pin 10, had to switch to an analog capable pin again
#define PIN_BUTTON_CENTER   2


#define FORCE_GPS_FIX true  //set to true for indoor testing (skips over the part where it waits for a fix)
#define FORCE_RECORDING_STATE_SWITCH false   //switches the currentlyRecording flag every time the lock screen is entered so I can test that
                                            //the state machine properly hands off to the correct UI page based on recording state
#define STOP_RECORDING_IF_FIX_LOST false    //set to true if you want to stop the record if the GPS is enabled and the fix is lost.
                                            //set false if you want to keep recording in spite of lost GPS fix.

#define NUM_SENSORS 11      //number of sensors I'll be logging data with
#define INDICATOR_MAX_BRIGHTNESS 8

//note that the TX and RX pins (0 and 1 respectively) are used for the GPS serial
// define hardware serial port for GPS?
#define GPSSerial Serial1
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

///*
//create an instance of the RTC (sd featherwing) and build a char array for the days of the week
RTC_PCF8523 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
DateTime currentTime;       //real time according to RTC module (not GPS rtc) 
DateTime displayedTime;   //used for writing time on displaying and comparing to actual time so I can update screen in nonblocking manner
//*/

//define the builtin neopixel that I'll use as a state indicator for the system
Adafruit_NeoPixel indicator = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);   
//defining colors I'll use to indicate different system statuses
const uint32_t red = indicator.gamma32(indicator.ColorHSV(0, 255, 255));            //Planning to use to indicate no SD card installed / SD error
const uint32_t yellow = indicator.gamma32(indicator.ColorHSV(10900, 255, 255));     //Solid: waiting for first fix. Flashing: lost fix (not yet implemented)
const uint32_t green = indicator.gamma32(indicator.ColorHSV(21800, 255, 255));      //GPS fix, recording data to SD/Serial monitor
const uint32_t teal = indicator.gamma32(indicator.ColorHSV(32700, 255, 255));       //might be a good indicator for GPS fix, not yet recording
const uint32_t blue = indicator.gamma32(indicator.ColorHSV(43700, 255, 255));
const uint32_t pink = indicator.gamma32(indicator.ColorHSV(54600, 255, 255));       //GPS fix lost. 

//create the display object
Adafruit_SharpMem display(DISPLAY_CLK, DISPLAY_DI, DISPLAY_CS, 400, 240);

//creating utility functions
float metersToFeet(float meters) {return meters*3.28084;}
float celsiusToFahrenheit(float celsius) {return celsius*9/5 + 32;}

//define display graphic primitive constants
const int roundRectCornerRad = 4;
const int mainButtonMargin = 10;
const int mainButtonWidth = (display.width() - 4*mainButtonMargin) / 3;
const int mainButtonHeight = (display.height() - 2*mainButtonMargin) / 4;


//make the enum that will handle the UI page states. 
//Main page (not recording), recording, setup, lock screen 

enum masterUIState
{
    mainPage,                   //on restart, not recording. has 3 buttons: setup, record, lock screen. Maybe also power off/sleep? Dont' know how to do that one yet.
    setupPage,                  //choose which sensors to enable for logging
    recordingPage,              //the page that displays when actively recording. has two buttons: stop recording, lock screen.
    recordingWaitingPage,       //the page that shows when user starts recording and GPS is enabled, but no GPS fix is available yet.
    lockPage                    //show some info (e.g., recording in progress, altitude, whatever). One button: press and hold 3 seconds to unlock.
};

//feel like I may need other sub-enums to deal with button states on each page for driving display. need to highlight button selections, etc.
//e.g.,

enum displayButtons    //states for properly tracking and highlighting the buttons on all display pages
{
    //thought it seemed simpler in long run to have these all be part of the same enum
    //buttons on the main page
    setupSensors,                   //main page left button
    startRecording,                 //main page center button
    lockScreen,                     //main page right button
    goToSleep,                      //unused, but saving this option for later if I can figure out how to make the mcu sleep
    back,                           //reuse this for all the secondary pages
    stopRecording,                  //displayed on recording screen
    unlockScreen,                   //displayed on lock screen
    selectRecordMode,               //display on setup screen
    gpsSetupUIButton,               //the following are used for tracking which button is highlighted on the display
    imuSetupUIButton,
    humiditySetupUIButton,
    barometerSetupUIButton,
    tempSetupUIButton,
    altimeterSetupUIButton,
    uvSetupUIButton,
    particulateSetupUIButton,
    tdsSetupUIButton,
    turbiditySetupUIButton,
    waterTempSetupUIButton,
    noSensorUISelection                          //going to use this for my subfunction that draws the sensor setup buttons (sometimes none of those will be selected)
};

//enum for recording mode flags and other useful state flags
enum recordingModeFlags
{
    recordingSteady,
    recordingBurst
};

//button press tracking flags
enum navButton      //going to use the ANO nav wheel, but leave out the encoder for now. 
{
    noPress,        //no buttons currently pressed
    leftSingleClick,           //use these as a group of flags to track which buttons are pressed
    rightSingleClick,          //pass these to functions to control how the display functions react
    upSingleClick,             //currently all for Single Click. Can make another group of these within the enum for double clicks and long press
    downSingleClick,
    centerSingleClick,
    centerLongPress
};

//going to try using a struct to keep track of which sensors are enabled so I can know what information to log
//remember that I currently have 11 sensors
struct sensorEnableFlags
{
    bool GPS;               //turn all of GPS logging on or off (choose what data from GPS to log elsewhere)
    bool IMU;               //don't know what I'd record from this yet, maybe inclination
    bool humidity;          
    bool barometer;         //barometer from BMP280
    bool temp;              //air temp from BMP280
    bool altimeter;         //calibrated altitude from BMP280. redundant with GPS
    bool UV;
    bool particulate;       //particulate matter sensor for air quality
    bool TDS;               //total disolved solids (water)
    bool turbidity;         //water
    bool waterTemp;         //water temp sensor
};

//set the default sensors to record
sensorEnableFlags sensors = 
{
    true,                   //gps on by default
    false,                  
    true,                   //humidity on by default
    true,                   //barometer on by default
    true,                   //air temp on by default
    false,
    false,                   //I want UV on by default, but keep it off for now until I actually have the sensor
    false,
    false,
    false,
    false
};






//create the labels and arrays of button objects for each UI page
char mainPageButtonLabels[3][7] = {"Setup", "Record", "Lock"};  //The [7] is because each of these labels is an array of chars, not a string like I'm used to from Python
Adafruit_GFX_Button mainPageButtons[3];                         //the actual array of buttons for the main page
char recordingPageButtonLabels[2][5] = {"Stop", "Lock"};
Adafruit_GFX_Button recordingPageButtons[2];
Adafruit_GFX_Button backButton;         //just a single button object that I'll reuse for all back buttons. Keeps them at consistent position across UI if I do this.
Adafruit_GFX_Button unlockButton;       //requires long click to unlock
Adafruit_GFX_Button recordSteadyButton;   //for selecting continuous or burst mode
Adafruit_GFX_Button recordBurstButton;

//create buttons and labels for the sensors
Adafruit_GFX_Button gpsButton;
Adafruit_GFX_Button imuButton;
Adafruit_GFX_Button humidityButton;
Adafruit_GFX_Button barometerButton;
Adafruit_GFX_Button tempButton;
Adafruit_GFX_Button altimeterButton;
Adafruit_GFX_Button uvButton;
Adafruit_GFX_Button particulateButton;
Adafruit_GFX_Button tdsButton;
Adafruit_GFX_Button turbidityButton;
Adafruit_GFX_Button waterTempButton;



//create some standard values for sensor button positions and sizes
int sensorButtonWidth = (display.width() - 4*mainButtonMargin) / 3;
int sensorButtonHeight = (display.height() - 7*mainButtonMargin) / 6;

//create a function for drawing an X over sensor buttons to show that they're disabled
void drawSensorDisableX(int x, int y)
{
    display.drawLine(x + 10, y + 10, x + sensorButtonWidth - 10, y + sensorButtonHeight - 10, BLACK);
    display.drawLine(x + sensorButtonWidth - 10, y + 10, x + 10, y + sensorButtonHeight - 10, BLACK);
}


//flags for tracking the main states of the system and the UI
masterUIState currentUIPage = mainPage;                 //initialize the UI state to the main page, since this is restart/boot
displayButtons displayButtonSelection = setupSensors;   //initialize button highlighting/tracking to sensor setup button on main page for boot
navButton navigationButton = noPress;                   //initialize state of button presses to none pressed. 
bool currentlyRecording = false;                        //set to true when recording is started, set to false when stopped.
recordingModeFlags recordMode = recordingSteady;        //default to steady record mode. Can also set to recordingBurst


void drawDateTimeUIElement(int x, int y)
{
    display.setCursor(x, y);
    display.setTextSize(2);
    display.setTextColor(BLACK);
    display.println(daysOfTheWeek[displayedTime.dayOfTheWeek()]);
    display.setCursor(x, y+20);
    displayedTime.hour() > 12 ? display.print(displayedTime.hour() - 12) : display.print(displayedTime.hour());
    display.print(":");
    if (displayedTime.minute() < 10) {display.print("0"); display.print(displayedTime.minute());} else display.print(displayedTime.minute());
    if (displayedTime.hour() > 11) display.print("p");
    display.print(", ");
    display.print(displayedTime.month());
    display.print("/");
    display.println(displayedTime.day());
}

displayButtons drawSensorSetupButtons(displayButtons selected)
{
    /*possible values of selected button that will be passed in:
    gpsSetupUIButton, imuSetupUIButton, humiditySetupUIButton, barometerSetupUIButton, tempSetupUIButton, altimeterSetupUIButton,
    uvSetupUIButton, particulateSetupUIButton, tdsSetupUIButton, turbiditySetupUIButton, waterTempSetupUIButton, noSensorUISelection*/

    //if the sensor is enabled, the button needs to be filled in. if disabled, not filled in.
    //additionally, if the button is selected, it needs to be filled in. however if it's enabled and selected, it will already be filled in.
    //I think I need a clear selector cursor or something, but with the way I've set that up, that will be hard. So for now, nonoptimal solution.
    //first, draw all buttons filled or not based on enabled state. Then, draw the selected buttons over those. I think that should work fine. 

    sensors.altimeter ? altimeterButton.drawButton(true) : altimeterButton.drawButton(false);   //draw filled in if enabled, else false. Ternary operator.
    sensors.barometer ? barometerButton.drawButton(true) : barometerButton.drawButton(false);
    sensors.GPS ? gpsButton.drawButton(true) : gpsButton.drawButton(false);
    sensors.humidity ? humidityButton.drawButton(true) : humidityButton.drawButton(false);
    sensors.IMU ? imuButton.drawButton(true) : imuButton.drawButton(false);
    sensors.particulate ? particulateButton.drawButton(true) : particulateButton.drawButton(false);
    sensors.TDS ? tdsButton.drawButton(true) : tdsButton.drawButton(false);
    sensors.temp ? tempButton.drawButton(true) : tempButton.drawButton(false);
    sensors.turbidity ? turbidityButton.drawButton(true) : turbidityButton.drawButton(false);
    sensors.UV ? uvButton.drawButton(true) : uvButton.drawButton(false);
    sensors.waterTemp ? waterTempButton.drawButton(true) : waterTempButton.drawButton(false);

    switch (selected)
    {
    case noSensorUISelection:
        break;
    case altimeterSetupUIButton:
        altimeterButton.drawButton(true);
        break;
    case gpsSetupUIButton:
        gpsButton.drawButton(true);
        break;
    case imuSetupUIButton:
        imuButton.drawButton(true);
        break;
    case humiditySetupUIButton:
        humidityButton.drawButton(true);
        break;
    case barometerSetupUIButton:
        barometerButton.drawButton(true);
        break;
    case tempSetupUIButton:
        tempButton.drawButton(true);
        break;
    case uvSetupUIButton:
        uvButton.drawButton(true);
        break;
    case particulateSetupUIButton:
        particulateButton.drawButton(true);
        break;
    case tdsSetupUIButton:
        tdsButton.drawButton(true);
        break;
    case turbiditySetupUIButton:
        turbidityButton.drawButton(true);
        break;
    case waterTempSetupUIButton:
        waterTempButton.drawButton(true);
        break;
    }

}


//the main page of the display when not recording.
//have a different function track nav button presses and tell this function which button to  hightlight.
//this function also returns which button it currently has selected
displayButtons drawMainPage(bool initialClear, bool refresh, displayButtons selectedButton)
{
    if (initialClear) display.clearDisplay();
    //Want three menu options: Setup, Begin Recording, Lock Screen.
    //I think I'll have those three buttons along the bottom, with a window at the top for displaying useful data (altitude, time, temp, gps fix state, etc)

    display.setCursor(10,10);
    display.setTextSize(3);
    display.setTextColor(BLACK);
    display.println("Main Menu.");

    drawDateTimeUIElement(250, 10);

    switch (selectedButton)
    {
        case setupSensors:
            //highlight sensors button
            mainPageButtons[0].drawButton(true);    //highlight selected button (setup)
            mainPageButtons[1].drawButton(false);
            mainPageButtons[2].drawButton(false);
            break;
        case startRecording:
            //highlight start recording button
            mainPageButtons[0].drawButton(false);    
            mainPageButtons[1].drawButton(true);    //highlight selected button (record)
            mainPageButtons[2].drawButton(false);
            break;
        case lockScreen:
            //highlight lock screen button
            mainPageButtons[0].drawButton(false);    
            mainPageButtons[1].drawButton(false);
            mainPageButtons[2].drawButton(true);    //highlight selected button (lock)
            break;
    }

    if (refresh) display.refresh();
    return selectedButton;
}



displayButtons drawSensorSetupPage(bool initialClear, bool refresh, displayButtons selectedButton)
{
    //placeholder 
    if (initialClear) display.clearDisplay();
    display.setCursor(display.width() - mainButtonMargin - mainButtonWidth + 20, mainButtonMargin);
    display.setTextSize(3);
    display.setTextColor(BLACK);
    display.println("Mode:");

    if (selectedButton == back || selectedButton == selectRecordMode)
    {
        drawSensorSetupButtons(noSensorUISelection);

        switch (selectedButton)
        {
            case back:
                backButton.drawButton(true); //draw highlighted. It's the default button to be selected when going to this page
                if (recordMode == recordingSteady) recordSteadyButton.drawButton(false);
                else if (recordMode == recordingBurst) recordBurstButton.drawButton(false);
                break;
            case selectRecordMode:
                backButton.drawButton(false);
                if (recordMode == recordingSteady) recordSteadyButton.drawButton(true);
                else if (recordMode == recordingBurst) recordBurstButton.drawButton(true);
        }
    }
    else
    {
        drawSensorSetupButtons(selectedButton);         //draw the highlighted sensor button
        backButton.drawButton(false);                   
        if (recordMode == recordingSteady) recordSteadyButton.drawButton(false);
        else recordBurstButton.drawButton(false);

    }
    
    if (refresh) display.refresh();
    return selectedButton;
}

//draw the actively recording page
displayButtons drawRecordingPage(bool initialClear, bool refresh, displayButtons selectedButton)
{

    if (initialClear) display.clearDisplay();
    display.setCursor(10,10);
    display.setTextSize(3);
    display.setTextColor(BLACK);
    display.println("Recording.");

    drawDateTimeUIElement(250, 10);

    switch (selectedButton)
    {
        case stopRecording:
            recordingPageButtons[0].drawButton(true);   //draw the "stop recording button" inverted to show it's selected
            recordingPageButtons[1].drawButton(false);
            break;
        case lockScreen:
            recordingPageButtons[0].drawButton(false);
            recordingPageButtons[1].drawButton(true);   //draw the lock screen button inverted
            break;
    }

    if (refresh) display.refresh();
    return selectedButton;
}

//draw the recording page that shows when GPS is enabled but no fix is yet available
displayButtons drawRecordingWaitingPage(bool initialClear, bool refresh, displayButtons selectedButton)
{
    if (initialClear) display.clearDisplay();
    display.setCursor(10, display.height() / 2 - 30);
    display.setTextSize(3);
    display.setTextColor(BLACK);
    display.println("Waiting for GPS fix.");
    drawDateTimeUIElement(250, 10);
    //for now, decided to remove option to lock screen on the waiting for GPS fix and waiting to record page. complicated things too much.
    recordingPageButtons[0].drawButton(true);           //draw the stop recording button selected
    if (refresh) display.refresh();
    return selectedButton;
}


displayButtons drawLockPage(bool initialClear, bool refresh, displayButtons selectedButton)
{

    if (initialClear) display.clearDisplay();
    display.setCursor(10,10);
    display.setTextSize(3);
    display.setTextColor(BLACK);
    display.println("Locked.");
    
    drawDateTimeUIElement(250, 10);

    unlockButton.drawButton(true);      //only the unlock button is available, so always draw it, always highlight

    if (refresh) display.refresh();
    return selectedButton;
}

//initialize the button objects for navigation buttons
OneButton buttonLeft = OneButton(PIN_BUTTON_LEFT, true, true);
OneButton buttonRight = OneButton(PIN_BUTTON_RIGHT, true, true);
OneButton buttonCenter = OneButton(PIN_BUTTON_CENTER, true, true);
OneButton buttonUp = OneButton(PIN_BUTTON_UP, true, true);
OneButton buttonDown = OneButton(PIN_BUTTON_DOWN, true, true);

void updateNavButtons(void)
{
    //Call this function every time through main loop to update the button states and check for presses.
    //Enables me to set a single flag (defined below as the navigationButton variable) to track button presses.
    //Be sure to reset the flag to noPress after each use.
    //Doesn't even have to return a value, that's done by the attachClick method in the Setup().
    //Just using this to consolidate all the button.tick() calls.
    buttonLeft.tick();
    buttonCenter.tick();
    buttonRight.tick();
    buttonDown.tick();
    buttonUp.tick();
}




void UIStateManager(void)
{
    //Only call this function when there has been a nav button press. If there's no button press, the main loop will update the display
    //every second or so with relevant information.

    //At first, this will read and modify global flag variables that hold the state of the state machine. However, in the long term,
    //I want to pass the nav button, display button, and master state flags as arguments, and have the function directly modify
    //the master state and the proper display button to highlight when first switching to a new screen. I think that requries
    //passing arguments by reference, perhaps, so that changing those values within the function can change the actual flag variables
    //directly. I just don't know how to do that yet, so for now, I'll just reference the global flags.

    switch (currentUIPage) //first figure out what UI page I'm on (main, setup, recording, lock)
    {
        case mainPage:  //if I'm on the main page, determine which display button to highlight
            switch (displayButtonSelection) //look at the current button selection so that I can know where to move the button based on the nav direction
            {
                case setupSensors:
                    if (navigationButton == rightSingleClick) //move highlighting from setup sensors to start recording
                    {
                        displayButtonSelection = drawMainPage(true, true, startRecording); //trying to shortcut a bit, might need to split into two lines
                    }
                    else if (navigationButton == centerSingleClick) //change master page to the sensor setup page
                    {
                        currentUIPage = setupPage;
                        displayButtonSelection = drawSensorSetupPage(true, true, back); //the default highlighted button on the setup page is "back"
                    }
                    else if (navigationButton == leftSingleClick) displayButtonSelection = drawMainPage(true, true, lockScreen);
                    break;
                case startRecording:
                    if (navigationButton == rightSingleClick) displayButtonSelection = drawMainPage(true, true, lockScreen);
                    else if (navigationButton == leftSingleClick) displayButtonSelection = drawMainPage(true, true, setupSensors);
                    //if I tell the UI to start recording                    
                    else if (navigationButton == centerSingleClick)
                    {
                        if (sensors.GPS && (! GPS.fix))    //if gps is enabled but there's no fix yet, go to the recordingWaitingPage
                        {
                            currentUIPage = recordingWaitingPage;
                            currentlyRecording = false;
                            displayButtonSelection = drawRecordingWaitingPage(true, true, stopRecording);   //only this one button available on that page
                        }
                        else    //either gps isn't enabled, or it is and there's a fix
                        {
                            currentUIPage = recordingPage;
                            currentlyRecording = true;  //set the currently recording flag to true to indicate to rest of program to start recording
                            displayButtonSelection = drawRecordingPage(true, true, lockScreen);
                        }
                        
                    }
                    break;
                case lockScreen:
                    if (navigationButton == leftSingleClick) displayButtonSelection = drawMainPage(true, true, startRecording);
                    else if (navigationButton == centerSingleClick)
                    {
                        currentUIPage = lockPage;
                        displayButtonSelection = drawLockPage(true, true, unlockScreen);
                    }
                    else if (navigationButton == rightSingleClick) displayButtonSelection = drawMainPage(true, true, setupSensors);
                    break;
                //later I can add a "go to sleep button," which I'll have to include here.
            }
            navigationButton = noPress; //once I've made it here, changes have been made to the UI. reset the navigationButton flag
            break;
        case setupPage:
            //temporary code. just want to see the back button work.
            switch (displayButtonSelection)
            {
                case back:      //if the back button is highlighted
                    if (navigationButton == centerSingleClick)
                    {
                        currentUIPage = mainPage;
                        displayButtonSelection = drawMainPage(true, true, setupSensors);
                    }
                    else if (navigationButton == downSingleClick || navigationButton == upSingleClick)
                    {
                        //currentUIPage = setupPage;
                        displayButtonSelection = drawSensorSetupPage(true, true, selectRecordMode);
                    }
                    else if (navigationButton == leftSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, waterTempSetupUIButton);
                    else if (navigationButton == rightSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, altimeterSetupUIButton);
                    break;
                case selectRecordMode:
                    if (navigationButton == downSingleClick || navigationButton == upSingleClick)
                    {
                        currentUIPage = setupPage;
                        displayButtonSelection = drawSensorSetupPage(true, true, back);
                    }
                    else if (navigationButton == centerSingleClick)
                    {
                        currentUIPage = setupPage;
                        if (recordMode == recordingSteady) recordMode = recordingBurst;
                        else if (recordMode == recordingBurst) recordMode = recordingSteady;
                        displayButtonSelection = drawSensorSetupPage(true, true, selectRecordMode);
                    }
                    else if (navigationButton == rightSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, humiditySetupUIButton);
                    else if (navigationButton == leftSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, imuSetupUIButton);
                    break;
                case gpsSetupUIButton:
                    if (navigationButton == centerSingleClick) 
                    {
                        sensors.GPS = !sensors.GPS; //switch enabled state of the GPS sensor enable flag
                        displayButtonSelection = drawSensorSetupPage(true, true, gpsSetupUIButton);  
                    }
                    else if (navigationButton == rightSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, particulateSetupUIButton);
                    else if (navigationButton == leftSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, selectRecordMode);
                    else if (navigationButton == upSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, uvSetupUIButton);
                    else if (navigationButton == downSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, humiditySetupUIButton);
                    break;
                case imuSetupUIButton:
                    if (navigationButton == centerSingleClick) 
                    {
                        sensors.IMU = !sensors.IMU; //switch enabled state of the GPS sensor enable flag
                        displayButtonSelection = drawSensorSetupPage(true, true, imuSetupUIButton);  
                    }
                    else if (navigationButton == rightSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, selectRecordMode);
                    else if (navigationButton == leftSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, humiditySetupUIButton);
                    else if (navigationButton == upSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, particulateSetupUIButton);
                    else if (navigationButton == downSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, tdsSetupUIButton);
                    break;
                case humiditySetupUIButton:
                    if (navigationButton == centerSingleClick) 
                    {
                        sensors.humidity = !sensors.humidity; //switch enabled state of the GPS sensor enable flag
                        displayButtonSelection = drawSensorSetupPage(true, true, humiditySetupUIButton);  
                    }
                    else if (navigationButton == rightSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, imuSetupUIButton);
                    else if (navigationButton == leftSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, selectRecordMode);
                    else if (navigationButton == upSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, gpsSetupUIButton);
                    else if (navigationButton == downSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, barometerSetupUIButton);
                    break;
                case barometerSetupUIButton:
                    if (navigationButton == centerSingleClick) 
                    {
                        sensors.barometer = !sensors.barometer; //switch enabled state of the GPS sensor enable flag
                        displayButtonSelection = drawSensorSetupPage(true, true, barometerSetupUIButton);  
                    }
                    else if (navigationButton == rightSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, tdsSetupUIButton);
                    else if (navigationButton == leftSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, selectRecordMode);
                    else if (navigationButton == upSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, humiditySetupUIButton);
                    else if (navigationButton == downSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, tempSetupUIButton);
                    break;
                case tempSetupUIButton:
                    if (navigationButton == centerSingleClick) 
                    {
                        sensors.temp = !sensors.temp; //switch enabled state of the GPS sensor enable flag
                        displayButtonSelection = drawSensorSetupPage(true, true, tempSetupUIButton);  
                    }
                    else if (navigationButton == rightSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, turbiditySetupUIButton);
                    else if (navigationButton == leftSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, back);
                    else if (navigationButton == upSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, barometerSetupUIButton);
                    else if (navigationButton == downSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, altimeterSetupUIButton);
                    break;
                case altimeterSetupUIButton:    //should be finished with all five buttons here.
                    if (navigationButton == centerSingleClick)
                    {
                        sensors.altimeter = !sensors.altimeter;
                        displayButtonSelection = drawSensorSetupPage(true, true, altimeterSetupUIButton);
                    }
                    else if (navigationButton == rightSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, waterTempSetupUIButton);
                    else if (navigationButton == leftSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, back);
                    else if (navigationButton == upSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, tempSetupUIButton);
                    else if (navigationButton == downSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, uvSetupUIButton);
                    break;
                case uvSetupUIButton:
                    if (navigationButton == centerSingleClick) 
                    {
                        sensors.UV = !sensors.UV; //switch enabled state of the GPS sensor enable flag
                        displayButtonSelection = drawSensorSetupPage(true, true, uvSetupUIButton);  
                    }
                    else if (navigationButton == rightSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, waterTempSetupUIButton);
                    else if (navigationButton == leftSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, back);
                    else if (navigationButton == upSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, altimeterSetupUIButton);
                    else if (navigationButton == downSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, gpsSetupUIButton);
                    break;
                case particulateSetupUIButton:
                    if (navigationButton == centerSingleClick) 
                    {
                        sensors.particulate = !sensors.particulate; //switch enabled state of the GPS sensor enable flag
                        displayButtonSelection = drawSensorSetupPage(true, true, particulateSetupUIButton);  
                    }
                    else if (navigationButton == rightSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, selectRecordMode);
                    else if (navigationButton == leftSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, gpsSetupUIButton);
                    else if (navigationButton == upSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, waterTempSetupUIButton);
                    else if (navigationButton == downSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, imuSetupUIButton);
                    break;
                case tdsSetupUIButton:
                    if (navigationButton == centerSingleClick) 
                    {
                        sensors.TDS = !sensors.TDS; //switch enabled state of the GPS sensor enable flag
                        displayButtonSelection = drawSensorSetupPage(true, true, tdsSetupUIButton);  
                    }
                    else if (navigationButton == rightSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, selectRecordMode);
                    else if (navigationButton == leftSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, barometerSetupUIButton);
                    else if (navigationButton == upSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, imuSetupUIButton);
                    else if (navigationButton == downSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, turbiditySetupUIButton);
                    break;
                case turbiditySetupUIButton:
                    if (navigationButton == centerSingleClick) 
                    {
                        sensors.turbidity = !sensors.turbidity; //switch enabled state of the GPS sensor enable flag
                        displayButtonSelection = drawSensorSetupPage(true, true, turbiditySetupUIButton);  
                    }
                    else if (navigationButton == rightSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, back);
                    else if (navigationButton == leftSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, tempSetupUIButton);
                    else if (navigationButton == upSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, tdsSetupUIButton);
                    else if (navigationButton == downSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, waterTempSetupUIButton);
                    break;
                case waterTempSetupUIButton:    //should be finished with all five buttons here.
                    if (navigationButton == centerSingleClick)
                    {
                        sensors.waterTemp = !sensors.waterTemp;
                        displayButtonSelection = drawSensorSetupPage(true, true, waterTempSetupUIButton);
                    }
                    else if (navigationButton == rightSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, back);
                    else if (navigationButton == leftSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, altimeterSetupUIButton);
                    else if (navigationButton == upSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, turbiditySetupUIButton);
                    else if (navigationButton == downSingleClick) displayButtonSelection = drawSensorSetupPage(true, true, particulateSetupUIButton);
                    break;
            }
            navigationButton = noPress;     //if I forget this line, the menu immediately jumps back to the page I was trying to leave.
            break;
        case recordingPage:
            switch (displayButtonSelection)
            {
                case stopRecording:
                    if (navigationButton == rightSingleClick || navigationButton == leftSingleClick) displayButtonSelection = drawRecordingPage(true, true, lockScreen);
                    else if (navigationButton == centerSingleClick) 
                    {
                        currentUIPage = mainPage;
                        currentlyRecording = false;     //set the currentlyRecording flag to false to indicated to rest of program to stop record
                        displayButtonSelection = drawMainPage(true, true, setupSensors);
                    }
                    break;
                case lockScreen:
                    if (navigationButton == rightSingleClick || navigationButton == leftSingleClick) displayButtonSelection = drawRecordingPage(true, true, stopRecording);
                    else if (navigationButton == centerSingleClick) 
                    {
                        currentUIPage = lockPage;
                        displayButtonSelection = drawLockPage(true, true, unlockScreen);
                    }    
                    break;
            }
            navigationButton = noPress;
            break;
        case recordingWaitingPage:
            if (navigationButton == centerSingleClick)
            {
                currentUIPage = mainPage;
                currentlyRecording = false;
                displayButtonSelection = drawMainPage(true,true, setupSensors);
            }
            
            navigationButton = noPress;
            break;
        case lockPage:
            if (navigationButton == centerLongPress)
            {
                if (currentlyRecording)     //if recording data, return to the recording page from the lock screen
                {
                    currentUIPage = recordingPage;
                    displayButtonSelection = drawRecordingPage(true, true, lockScreen);
                }
                
                else                        //if not recording, return to main menu
                {
                    currentUIPage = mainPage;   
                    displayButtonSelection = drawMainPage(true, true, setupSensors);
                }
                if (FORCE_RECORDING_STATE_SWITCH) currentlyRecording = !currentlyRecording;
            }
            navigationButton = noPress;
            break;
    }
}



uint32_t timer = millis();   //used for timing the updates on the Serial monitor and the logfile
unsigned long lastDisplayUpdate;
unsigned long lastRTCread = millis();   //keep track of when the RTC was last read from. Not sure if I need this yet.
            



//******************************************************************************//



void setup()
{
    //attach the proper functions to the navigation buttons
    buttonLeft.attachClick([]() {navigationButton = leftSingleClick;});      //Lambda function! 
    buttonCenter.attachClick([]() {navigationButton = centerSingleClick;});
    buttonCenter.attachLongPressStart([]() {navigationButton = centerLongPress;});  //long press for unlocking screen
    buttonCenter.setPressTicks(1600);   //set to 1.6 seconds for long press
    buttonRight.attachClick([]() {navigationButton = rightSingleClick;});
    buttonUp.attachClick([]() {navigationButton = upSingleClick;});
    buttonDown.attachClick([]() {navigationButton = downSingleClick;});

    //initialize the UI buttons
    //hardcoding some positions for the recording page buttons as a test on the recording page.
    //may just keep hardcoding for now. the UI layout won't change so frequently that it's worth making a more flexible system
    recordingPageButtons[0].initButtonUL(&display, mainButtonMargin, display.height() - mainButtonMargin - mainButtonHeight, mainButtonWidth, 
                                            mainButtonHeight, BLACK, WHITE, BLACK, recordingPageButtonLabels[0], 3);
    recordingPageButtons[1].initButtonUL(&display, 3*mainButtonMargin + 2*mainButtonWidth, display.height() - mainButtonMargin - mainButtonHeight,
                                            mainButtonWidth, mainButtonHeight, BLACK, WHITE, BLACK, recordingPageButtonLabels[1], 3);
    //initialize the main page UI buttons. For loop made more sense here
    for (int i = 0; i < 3; i++) 
    {
        mainPageButtons[i].initButtonUL(&display, mainButtonMargin + i*mainButtonMargin + i*mainButtonWidth, display.height() - mainButtonMargin - mainButtonHeight,
                                        mainButtonWidth, mainButtonHeight, BLACK, WHITE, BLACK, mainPageButtonLabels[i], 3);
    }
    //initialize back and unlock buttons shared across multiple pages
    backButton.initButtonUL(&display, 3*mainButtonMargin + 2*mainButtonWidth, display.height() - mainButtonMargin - mainButtonHeight, 
                            mainButtonWidth, mainButtonHeight, BLACK, WHITE, BLACK, "Back", 3);
    unlockButton.initButtonUL(&display, 3*mainButtonMargin + 2*mainButtonWidth, display.height() - mainButtonMargin - mainButtonHeight, 
                            mainButtonWidth, mainButtonHeight, BLACK, WHITE, BLACK, "Unlock", 3);
    //create buttons for record mode flags. The button that is displayed changes based on mode, which is why I have two nearly identical copies below.
    recordSteadyButton.initButtonUL(&display, display.width() - mainButtonMargin - mainButtonWidth, mainButtonMargin*2 + mainButtonHeight - 30, 
                                mainButtonWidth, mainButtonHeight, BLACK, WHITE, BLACK, "Steady", 3);
    recordBurstButton.initButtonUL(&display, display.width() - mainButtonMargin - mainButtonWidth, mainButtonMargin*2 + mainButtonHeight - 30, 
                                mainButtonWidth, mainButtonHeight, BLACK, WHITE, BLACK, "Burst", 3);


    //initialize the sensor enable buttons. there's got to be a more efficient way to do this... 
    //creating position variables for the sensor UI buttons to make things cleaner below
    int sensorPositionsCol1X = mainButtonMargin;
    int sensorPositionsCol2X = sensorButtonWidth + 2*mainButtonMargin;
    int sensorPositionsColsY[6];    //array of Y positions for the sensor UI buttons. just to make initializing the buttons below a bit cleaner.
    for (int i = 0; i < 6; i++)
    {
        sensorPositionsColsY[i] = mainButtonMargin + i*mainButtonMargin + i*sensorButtonHeight;
    }
    //do column 1 first
    gpsButton.initButtonUL(&display, sensorPositionsCol1X, sensorPositionsColsY[0], sensorButtonWidth, sensorButtonHeight, BLACK, WHITE, BLACK, "GPS", 2);
    humidityButton.initButtonUL(&display, sensorPositionsCol1X, sensorPositionsColsY[1], sensorButtonWidth, sensorButtonHeight, BLACK, WHITE, BLACK, "Humidity", 2);
    barometerButton.initButtonUL(&display, sensorPositionsCol1X, sensorPositionsColsY[2], sensorButtonWidth, sensorButtonHeight, BLACK, WHITE, BLACK, "Barometer", 2);
    tempButton.initButtonUL(&display, sensorPositionsCol1X, sensorPositionsColsY[3], sensorButtonWidth, sensorButtonHeight, BLACK, WHITE, BLACK, "Air Temp", 2);
    altimeterButton.initButtonUL(&display, sensorPositionsCol1X, sensorPositionsColsY[4], sensorButtonWidth, sensorButtonHeight, BLACK, WHITE, BLACK, "Altimeter", 2);
    uvButton.initButtonUL(&display, sensorPositionsCol1X, sensorPositionsColsY[5], sensorButtonWidth, sensorButtonHeight, BLACK, WHITE, BLACK, "UV Index", 2);
    //now column 2    
    particulateButton.initButtonUL(&display, sensorPositionsCol2X, sensorPositionsColsY[0], sensorButtonWidth, sensorButtonHeight, BLACK, WHITE, BLACK, "Air Qual", 2);
    imuButton.initButtonUL(&display, sensorPositionsCol2X, sensorPositionsColsY[1], sensorButtonWidth, sensorButtonHeight, BLACK, WHITE, BLACK, "IMU", 2);
    tdsButton.initButtonUL(&display, sensorPositionsCol2X, sensorPositionsColsY[2], sensorButtonWidth, sensorButtonHeight, BLACK, WHITE, BLACK, "H2O TDS", 2);
    turbidityButton.initButtonUL(&display, sensorPositionsCol2X, sensorPositionsColsY[3], sensorButtonWidth, sensorButtonHeight, BLACK, WHITE, BLACK, "H20 Turb", 2);
    waterTempButton.initButtonUL(&display, sensorPositionsCol2X, sensorPositionsColsY[4], sensorButtonWidth, sensorButtonHeight, BLACK, WHITE, BLACK, "H20 Temp", 2);


    //start the neopixel
    indicator.begin();
    indicator.setBrightness(INDICATOR_MAX_BRIGHTNESS);
    
    //while (!Serial);  // uncomment to have the sketch wait until Serial is ready. comment out if using away from computer
    Serial.begin(115200);
    Serial.println("Adafruit GPS library basic parsing test!");
    
    //start the display
    display.begin();
    display.clearDisplay();

    ///*
    //initialize the RTC and make sure it's good to go
    if (! rtc.begin())      //try to initialize the i2c connection to the RTC
    {
        display.setCursor(100, 100);
        display.setTextColor(BLACK);
        display.setTextSize(4);
        display.println("RTC NOT FOUND");
        delay(2000);
    }
    else if (! rtc.initialized() || rtc.lostPower())
    {
        display.setCursor(100, 100);
        display.setTextColor(BLACK);
        display.setTextSize(4);
        display.println("RTC NOT INITIALIZED");
        delay(2000);
    }
    //*/
    currentTime = rtc.now();
    displayedTime = currentTime;

    //start the SD card reading system
    while (!SD.begin(CS))
    {
        Serial.println("Card failed, or not present");
        indicator.setPixelColor(0, indicator.Color(255, 0, 0)); //set indicator to red
        indicator.show();
    }

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // Set the update rate (1hz recommended)
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);

    delay(1000);

    // Ask for firmware version
    GPSSerial.println(PMTK_Q_RELEASE);

//******************************************************************************//
/*
I may want to change when I actually start the GPS running and wait for a fix. Specifically, may want to move this into the main loop
and make it so that GPS start and wait for fix is triggered by enabling the GPS system in the Sensor Selection page of the UI. That means
I need to add another state or transition to the system that makes it so that I can't start recording data until a GPS fix is established.
I think that just means another substate system tracking GPS status and modifying interaction with the main page. I'll add that later though,
for now just want to get a basic UI running. 
*/
//******************************************************************************//


    // read data from the GPS to update the GPS.fix flag
    char c = GPS.read();

    //may be able to simplify this part to match what I have in the while loop below. fix later, this works for now
    if (GPS.newNMEAreceived()) 
    {
        Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
            return; // we can fail to parse a sentence in which case we should just wait for another
    }

    //set indicator to yellow to show waiting for first fix
    indicator.setPixelColor(0, yellow);
    indicator.show();

    //test the display functions here
    //drawMainPage(true, false, displayButtonSelection);

    display.setCursor(20,20);
    display.setTextSize(3);
    display.setTextColor(BLACK);
    display.println("Waiting for GPS fix.");
    display.refresh();

    //////////////////////////////////////////////////////////////////////////////////////////
    // FORCING GPS FIX SO I CAN TEST STATE MACHINE INDOORS//
    ///////////////////////////////////////////////////////////
    //if (FORCE_GPS_FIX) {GPS.fix = true; delay(1000);}
    delay(1000);
    //try this version of bypassing the waiting for fix period for now. going to move this block elsewhere later
    if (! FORCE_GPS_FIX)
    {
        while (!GPS.fix)  //this should break once a GPS fix is established.
        {
            indicator.setPixelColor(0, yellow);
            indicator.show();

            //read from the GPS stream to update the GPS.fix flag
            char c = GPS.read();
            if (GPS.newNMEAreceived()) 
            {
                //The following bit of code seems to update the GPS.fix flag (I think GPS.read() alone isn't enough.)
                //However, I don't know why this works yet.
                GPS.parse(GPS.lastNMEA());
            }
        }
    }
    //drawMainPage(true, true, displayButtonSelection);
    display.clearDisplay();
    display.setCursor(20,20);
    display.setTextSize(3);
    display.setTextColor(BLACK);
    FORCE_GPS_FIX ? display.println("Fix Bypassed.") : display.println("Fix Acquired.");
    display.refresh();
    delay(1000);
    
    drawMainPage(true, true, displayButtonSelection);
    lastDisplayUpdate = millis();
}

//******************************************************************************//

void loop() 
{
    //update rtc time 
    currentTime = rtc.now();
    
    static int first_run = 0;
    static int sd_failures = 0;
    static unsigned long write_num = 0;
     
    String dataString = "";     //the string of data that I'll write to each line of the logfile

    //call the function to poll the navigation buttons, update the master state machine and display accordingly
    updateNavButtons();
    if (navigationButton != noPress) {UIStateManager(); lastDisplayUpdate = millis();} //update the UI state if there was a button press and reset the display update timer.
    //Otherwise, if trying to record but waiting for GPS fix and fix suddenly acquired, switch from recordingWaitingPage to recordingPage
    else if (currentUIPage == recordingWaitingPage && GPS.fix)
    {
        displayedTime = currentTime;
        lastDisplayUpdate = millis();
        currentUIPage = recordingPage;
        displayButtonSelection = drawRecordingPage(true, true, lockScreen);
        currentlyRecording = true;
    }
    //If recording is active and GPS fix is lost, stop recording?
    else if (STOP_RECORDING_IF_FIX_LOST && currentlyRecording && sensors.GPS && (! GPS.fix))    //lost fix while recording GPS and want to stop recording if that happens
    {
        displayedTime = currentTime;
        lastDisplayUpdate = millis();
        currentlyRecording = false;         //stop recording
        currentUIPage = mainPage;
        displayButtonSelection = drawMainPage(true, true, setupSensors);
    }
    //Force the display to update the time when there's no button press. Time updates appropriately on button presses.
    else if (displayedTime.minute() != currentTime.minute())
    {
        displayedTime = currentTime;
        lastDisplayUpdate = millis();
        switch (currentUIPage)
        {
        case mainPage:
            drawMainPage(true, true, displayButtonSelection);               //call the state manager and it should update the time on the appropriate screen automatically
            break;
        case recordingPage:
            drawRecordingPage(true, true, displayButtonSelection);
            break;
        case recordingWaitingPage:
            drawRecordingWaitingPage(true, true, displayButtonSelection);
            break;
        case lockPage:
            drawLockPage(true, true, displayButtonSelection);
            break;
        default:
            break;
        }
    }



    //***********************************************************************//
    //***********************************************************************//
    /*
        I need to modify the GPS reading routine. Especially since the 'return' in the following if statements
        will skip everything after it in the main loop if it gets called. 
        I do need to make sure that I can actually parse good GPS data before reading it or trying to record it
        to SD, but I don't want to break the whole loop early.
    */
    //***********************************************************************//
    //***********************************************************************//

    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
        if (c) Serial.print(c);
            // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) 
    {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trying to print out data
        Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
            return; // we can fail to parse a sentence in which case we should just wait for another
            //NEED TO EXPLORE THE BEHAVIOR OF THIS RETURN, IT MIGHT CUT THE WHOLE LOOP SHORT
    }

    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > 2000) 
    {
       
        timer = millis(); // reset the timer
        Serial.print("\nTime: ");
        if (GPS.hour < 10) { Serial.print('0'); }
        Serial.print(GPS.hour, DEC); Serial.print(':');
        if (GPS.minute < 10) { Serial.print('0'); }
        Serial.print(GPS.minute, DEC); Serial.print(':');
        if (GPS.seconds < 10) { Serial.print('0'); }
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        if (GPS.milliseconds < 10) 
        {
            Serial.print("00");
        } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) 
        {
            Serial.print("0");
        }
        Serial.println(GPS.milliseconds);
        Serial.print("Date: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.println(GPS.year, DEC);
        Serial.print("Fix: "); Serial.print((int)GPS.fix);
        Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
        
        if (GPS.fix) 
        {
            if (first_run == 0) //create header for CSV
            {
                dataString = "Write_Num,Date,Time,Fix,Fix_Quality,Satellites,Lat,Long,Altitude,Angle";
                first_run = 1;
            }
            else
            {
                dataString = String(write_num) + "," + String(GPS.month) + "/" + String(GPS.day) + "/20" + String(GPS.year) + ","
                                + String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds) + ","
                                + String((int)GPS.fix) + "," + String((int)GPS.fixquality) + "," + String(GPS.satellites) + ","
                                + String(GPS.latitude) + String(GPS.lat) + "," + String(GPS.longitude) + String(GPS.lon) + ","
                                + String(GPS.altitude) + "," + String(GPS.angle);
                write_num++;
            }
        
            File dataFile = SD.open("datalog.txt", FILE_WRITE); //moving this into the loop that happens every 2 seconds seems to have fixed the crashing SD card problem
        
            if (dataFile)
            {
                dataFile.println(dataString);
                dataFile.close();
            }
            else
            {
                Serial.println("Could not open data file.");  
                sd_failures++;
                if (sd_failures > 5) //if unable to write to SD file more than 5 times, medium flash alternating red and blue to show SD card problem
                {
                    while (true)
                    {
                        indicator.setPixelColor(0, indicator.Color(255, 0, 0));
                        indicator.show();
                        delay(250);
                        indicator.setPixelColor(0, indicator.Color(0, 0, 255));
                        indicator.show();
                        delay(250);
                    }
                }
            }
            indicator.setPixelColor(0, green); //turn on the green indicator to show that there is a fix
            indicator.show();
            Serial.print("Location: ");
            Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
            Serial.print(", ");
            Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
            Serial.print("Speed (knots): "); Serial.println(GPS.speed);
            Serial.print("Angle: "); Serial.println(GPS.angle);
            Serial.print("Altitude: "); Serial.println(GPS.altitude);
            Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
        }
        else
        {
            indicator.setPixelColor(0, pink); //turn on pink indicator to show lost fix
            indicator.show();
        }
    }
}