/*

  macros.c - plugin for binding macros to aux input pins

  Part of grblHAL

  Public domain.

  Up to 4 macros can be bound to input pins by changing the N_MACROS symbol below.
  Each macro can be up to 127 characters long, blocks (lines) are separated by a vertical bar character: |
  Setting numbers $450 - $453 are for defining the macro content.
  Setting numbers $454 - $457 are for configuring which aux input port to assign to each macro.
  NOTES: If the driver does not support mapping of port numbers settings $454 - $457 will not be available.
         The mapped pins has to be interrupt capable and support falling interrupt mode.
         The controller must be in Idle mode when starting macros.

  Examples:
    $450=G0Y5|G1X0F50
    $451=G0x0Y0Z0

  Tip: use the $pins command to check the port mapping.
*/

#define N_MACROS 3 // MAX 4

#include <stdio.h>
#include <string.h>

#include "grbl/hal.h"
#include "grbl/nvs_buffer.h"
#include "grbl/nuts_bolts.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/platform.h"

#ifndef GRBL_BUILD
#error "grblHAL build 20211117 or later is required for this plugin!"
#endif

#if N_MACROS > 4
#undef N_MACROS
#define N_MACROS 4
#endif

static uint8_t val[N_MACROS] = {0};
static uint8_t prev_val[N_MACROS] = {99};
static uint8_t latch[N_MACROS] = {0};

//the index of this array must match the radio button descriptions
uint8_t cmd[] = {   NULL, 
                    CMD_CYCLE_START, 
                    CMD_FEED_HOLD, 
                    CMD_RESET, 
                    CMD_PROBE_CONNECTED_TOGGLE, 
                    CMD_OVERRIDE_SPINDLE_STOP, 
                    CMD_OVERRIDE_COOLANT_FLOOD_TOGGLE,
                    CMD_OVERRIDE_COOLANT_MIST_TOGGLE, 
                    CMD_OPTIONAL_STOP_TOGGLE, 
                    CMD_SINGLE_BLOCK_TOGGLE,
                    CMD_SAFETY_DOOR
                    };

typedef struct {
    uint32_t command_idx;
    char data[128];
} macro_setting_t;

typedef struct {
    macro_setting_t macro[N_MACROS];
} macro_settings_t;

static bool can_map_ports = false, is_executing = false;
static uint8_t n_ports;
static uint8_t port[N_MACROS];
static char max_port[4], *command;
static char rt_command;
static nvs_address_t nvs_address;
static on_report_options_ptr on_report_options;
static macro_settings_t plugin_settings;
static stream_read_ptr stream_read;
static driver_reset_ptr driver_reset;

static on_execute_realtime_ptr on_execute_realtime, on_execute_delay;

static uint32_t debounce_ms = 0;
static uint32_t polling_ms = 0;
#define DEBOUNCE_DELAY 250

static int16_t get_macro_char (void);

// Ends macro execution if currently running
// and restores normal operation.
static void end_macro (void)
{
    is_executing = false;
    if(hal.stream.read == get_macro_char) {
        hal.stream.read = stream_read;
        report_init_fns();
    }
}

// Called on a soft reset so that normal operation can be restored.
static void plugin_reset (void)
{
    end_macro();    // End macro if currently running.
    driver_reset(); // Call the next reset handler in the chain.
}

// Macro stream input function.
// Reads character by character from the macro and returns them when
// requested by the foreground process.
static int16_t get_macro_char (void)
{
    static bool eol_ok = false;

    if(*command == '\0') {                          // End of macro?
        end_macro();                                // If end reading from it
        return eol_ok ? SERIAL_NO_DATA : ASCII_LF;  // and return a linefeed if the last character was not a linefeed.
    }

    char c = *command++;    // Get next character.

    if((eol_ok = c == '|')) // If character is vertical bar
        c = ASCII_LF;       // return a linefeed character.

    return (uint16_t)c;
}

// This code will be executed after each command is sent to the parser,
// If an error is detected macro execution will be stopped and the status_code reported.
static status_code_t trap_status_report (status_code_t status_code)
{
    if(status_code != Status_OK) {
        char msg[30];
        sprintf(msg, "error %d in macro", (uint8_t)status_code);
        report_message(msg, Message_Warning);
        end_macro();
    }

    return status_code;
}

// Actual start of macro execution.
static void run_macro (uint_fast16_t state)
{   
    if(state == STATE_IDLE && hal.stream.read != get_macro_char) {
        stream_read = hal.stream.read;                      // Redirect input stream to read from the macro instead of
        hal.stream.read = get_macro_char;                   // the active stream. This ensures that input streams are not mingled.
        grbl.report.status_message = trap_status_report;    // Add trap for status messages so we can terminate on errors.
    }
}

// This resets the flag for debounce.
static void end_cmd (void)
{
    is_executing = false;
}

// Actual start of command execution.
static void run_cmd (uint_fast16_t state)
{
    grbl.enqueue_realtime_command(rt_command);
    end_cmd();
}

// On falling interrupt run macro if machine is in Idle state.
// Since this function runs in an interrupt context actual start of execution
// is registered as a single run task to be started from the foreground process.
// TODO: add debounce?
//ISR_CODE static void execute_macro (uint8_t irq_port, bool is_high)
//now uses polling.
static void execute_macro (uint8_t irq_port, bool is_high)
{
    uint32_t ms = hal.get_elapsed_ticks();
    if(ms < debounce_ms + DEBOUNCE_DELAY) // debounce
        return;    

    if(!is_high && !is_executing) {

        // Determine macro to run from port number
        uint_fast8_t idx = N_MACROS;
        do {
            idx--;
        } while(idx && port[idx] != irq_port);
        
        //if an RT command is selected execute it
        if(plugin_settings.macro[idx].command_idx){
            is_executing = true;
            debounce_ms = ms;
            rt_command = cmd[plugin_settings.macro[idx].command_idx];
            protocol_enqueue_rt_command(run_cmd);
            return;
        }else 
        //otherwise run the macro if idle
        if (state_get() == STATE_IDLE){
            is_executing = true;
            debounce_ms = ms;
            command = plugin_settings.macro[idx].data;
            if(!(*command == '\0' || *command == 0xFF))     // If valid command
                protocol_enqueue_rt_command(run_macro);     // register run_macro function to be called from foreground process.
        }        
    }
}

static void poll_buttons (void){
    
    uint32_t ms = hal.get_elapsed_ticks();
    if(ms < polling_ms + 50)
        return;

    int_fast8_t idx = N_MACROS;
    do {
        idx--;
        prev_val[idx] = val[idx];
        val[idx] = hal.port.wait_on_input(Port_Digital, port[idx], WaitMode_Immediate, 0.0f);//read the IO pin
        if ((prev_val[idx] == 0) && (val[idx] == 0) && (latch[idx] == 0)){
            latch[idx] = 1;
            execute_macro(port[idx], 0);
        } else{
            if ((prev_val[idx] == 1) && (val[idx] == 1))
                latch[idx] = 0;
        }
    } while(idx >= 0);

    polling_ms = ms;    
}

static void button_poll_realtime (sys_state_t grbl_state)
{
    on_execute_realtime(grbl_state);

    poll_buttons();
}

static void button_poll_delay (sys_state_t grbl_state)
{
    on_execute_delay(grbl_state);

    poll_buttons();
}

static const setting_group_detail_t macro_groups [] = {
    { Group_Root, Group_UserSettings, "Macros"}
};

static const setting_detail_t macro_settings[] = {
    { Setting_UserDefined_3, Group_UserSettings, "Macro 1", NULL, Format_String, "x(127)", "0", "127", Setting_NonCore, &plugin_settings.macro[0].data, NULL, NULL },
    { Setting_UserDefined_4, Group_UserSettings, "Macro 2", NULL, Format_String, "x(127)", "0", "127", Setting_NonCore, &plugin_settings.macro[1].data, NULL, NULL },
    { Setting_UserDefined_5, Group_UserSettings, "Macro 3", NULL, Format_String, "x(127)", "0", "127", Setting_NonCore, &plugin_settings.macro[2].data, NULL, NULL },

    { Setting_UserDefined_0, Group_UserSettings, "Macro 1 Button Function", NULL, Format_RadioButtons,  "Macro 1,Cycle Start,Pause,Halt,Probe Connected Toggle,Spindle Stop (during pause),Flood Coolant Toggle,Mist Coolant Toggle,Optional Stop Toggle,Single Block Mode Toggle, Parking Pause", NULL, NULL, Setting_NonCore, &plugin_settings.macro[0].command_idx, NULL, NULL },

    { Setting_UserDefined_1, Group_UserSettings, "Macro 2 Button Function", NULL, Format_RadioButtons,  "Macro 2,Cycle Start,Pause,Halt,Probe Connected Toggle,Spindle Stop (during pause),Flood Coolant Toggle,Mist Coolant Toggle,Optional Stop Toggle,Single Block Mode Toggle, Parking Pause", NULL, NULL, Setting_NonCore, &plugin_settings.macro[1].command_idx, NULL, NULL },

    { Setting_UserDefined_2, Group_UserSettings, "Macro 3 Button Function", NULL, Format_RadioButtons,  "Macro 3,Cycle Start,Pause,Halt,Probe Connected Toggle,Spindle Stop (during pause),Flood Coolant Toggle,Mist Coolant Toggle,Optional Stop Toggle,Single Block Mode Toggle, Parking Pause", NULL, NULL, Setting_NonCore, &plugin_settings.macro[2].command_idx, NULL, NULL },
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t macro_settings_descr[] = {
    { Setting_UserDefined_3, "Macro content for macro 1, separate blocks (lines) with the vertical bar character |." },

    { Setting_UserDefined_4, "Macro content for macro 2, separate blocks (lines) with the vertical bar character |." },

    { Setting_UserDefined_5, "Macro content for macro 3, separate blocks (lines) with the vertical bar character |." },

    { Setting_UserDefined_0, "Real-time function assigned to Macro 1 (overrides macro)"  },

    { Setting_UserDefined_1, "Real-time function assigned to Macro 2 (overrides macro)"  },

    { Setting_UserDefined_2, "Real-time function assigned to Macro 3 (overrides macro)"  },

};

#endif

// Write settings to non volatile storage (NVS).
static void macro_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(macro_settings_t), true);
}

// Restore default settings and write to non volatile storage (NVS).
static void macro_settings_restore (void)
{
    uint_fast8_t idx = N_MACROS, port = n_ports > N_MACROS ? n_ports - N_MACROS : 0;
    uint_fast8_t idy;

    char default_str[] = "G4P0";

    // Register empty macro strings and set default port numbers if mapping is available.
    for(idx = 0; idx < N_MACROS; idx++) {
            plugin_settings.macro[idx].command_idx = 0;
        for(idy = 0; idy < strlen(default_str); idy++) {
            plugin_settings.macro[idx].data[idy] = default_str[idy];
        };
        plugin_settings.macro[idx].data[idy] = '\0';
    };

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(macro_settings_t), true);
}

static void no_ports (uint_fast16_t state)
{
    report_message("Macro plugin failed to claim all needed ports!", Message_Warning);
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
static void macro_settings_load (void)
{
    uint_fast8_t idx = N_MACROS, n_ok = 0;

    if(hal.nvs.memcpy_from_nvs((uint8_t *)&plugin_settings, nvs_address, sizeof(macro_settings_t), true) != NVS_TransferResult_OK)
        macro_settings_restore();

    // If port mapping is available try to claim ports as configured.
    if(can_map_ports) {

        xbar_t *pin_info = NULL;

        //just claim them explicitly so that we can have unique names.
        idx--;
        port[idx] = n_ports-1;          
        if(hal.port.get_pin_info)
            pin_info = hal.port.get_pin_info(Port_Digital, Port_Input, port[idx]);
        if(pin_info && !(pin_info->cap.irq_mode & IRQ_Mode_Falling))                // Is port interrupt capable?
            port[idx] = 0xFF;                                                       // No, flag it as not claimed.
        else if(!ioport_claim(Port_Digital, Port_Input, &port[idx], "Macro 3 pin"))   // Try to claim the port.
            port[idx] = 0xFF;                                                       // If not successful flag it as not claimed.

        idx--;
        port[idx] = n_ports-2;          
        if(hal.port.get_pin_info)
            pin_info = hal.port.get_pin_info(Port_Digital, Port_Input, port[idx]);
        if(pin_info && !(pin_info->cap.irq_mode & IRQ_Mode_Falling))                // Is port interrupt capable?
            port[idx] = 0xFF;                                                       // No, flag it as not claimed.
        else if(!ioport_claim(Port_Digital, Port_Input, &port[idx], "Macro 2 pin"))   // Try to claim the port.
            port[idx] = 0xFF;    

        idx--;
        port[idx] = n_ports-3;          
        if(hal.port.get_pin_info)
            pin_info = hal.port.get_pin_info(Port_Digital, Port_Input, port[idx]);
        if(pin_info && !(pin_info->cap.irq_mode & IRQ_Mode_Falling))                // Is port interrupt capable?
            port[idx] = 0xFF;                                                       // No, flag it as not claimed.
        else if(!ioport_claim(Port_Digital, Port_Input, &port[idx], "Macro 1 pin"))   // Try to claim the port.
            port[idx] = 0xFF;                         
    }

    // Then try to register the interrupt handler.
    idx = N_MACROS;
    do {
        idx--;
        //if(port[idx] != 0xFF && hal.port.register_interrupt_handler(port[idx], IRQ_Mode_Falling, execute_macro))
            n_ok++;
    } while(idx);

    if(n_ok < N_MACROS)
        protocol_enqueue_rt_command(no_ports);
}

// Settings descriptor used by the core when interacting with this plugin.
static setting_details_t setting_details = {
    .groups = macro_groups,
    .n_groups = sizeof(macro_groups) / sizeof(setting_group_detail_t),
    .settings = macro_settings,
    .n_settings = sizeof(macro_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = macro_settings_descr,
    .n_descriptions = sizeof(macro_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = macro_settings_save,
    .load = macro_settings_load,
    .restore = macro_settings_restore
};

// Add info about our plugin to the $I report.
static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Macro plugin v0.02]" ASCII_EOL);
}

static void warning_msg (uint_fast16_t state)
{
    report_message("Macro plugin failed to initialize!", Message_Warning);
}

// A call macros_init will be issued automatically at startup.
void aux_macros_init (void)
{
    bool ok = (n_ports = ioports_available(Port_Digital, Port_Input)) > N_MACROS;

    if(ok && !(can_map_ports = ioport_can_claim_explicit())) {

        // Driver does not support explicit pin claiming, claim the highest numbered ports instead.

        uint_fast8_t idx = N_MACROS;

        do {
            idx--;
            if(!(ok = ioport_claim(Port_Digital, Port_Input, &port[idx], "Macro pin")))
                port[idx] = 0xFF;
        } while(idx);
    }

    // If enough free non volatile memory register our plugin with the core.
    if(ok && (nvs_address = nvs_alloc(sizeof(macro_settings_t)))) {

        // Register settings.
        settings_register(&setting_details);

        // Used for setting value validation.
        strcpy(max_port, uitoa(n_ports - 1));

        // Add our plugin to the $I options report.
        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        //add polling of button state to realtime and delay chains
        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = button_poll_realtime;

        on_execute_delay = grbl.on_execute_delay;
        grbl.on_execute_delay = button_poll_delay;        

        // Hook into the driver reset chain so we
        // can restore normal operation if a reset happens
        // when a macro is running.
        driver_reset = hal.driver_reset;
        hal.driver_reset = plugin_reset;
    } else
        protocol_enqueue_rt_command(warning_msg);
}