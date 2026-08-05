# daedalus_science
science control for daedulus robot

### Dynamixel Interface

Uhhhh thing that controls dynamixels for science package

## Dynamixel id list

Pump ids 1-6
Soil Collecto 7-8
Carousel 9
Gimbal 10

## Publishers

    1. send_status_pub:

## Subscribers

    1. carousel/curr_cuvette
    2. front_scoop/collector_speed
    3. back_scoop/collector_speed
    4. iso_front_in/milliliters
    5. iso_frount_out/milliliters
    6. iso_back_in/milliliters
    7. iso_back_out/milliliters
    8. hcl_pump/milliliters
    9. bayer_pump/milliliters
    10. gimbal_move_pos
    11. motor_error_status_confirmation_subscriber

## Carousel Callbacks

    1. cuvette_callback(self, msg)

## Scoop Callbacks

    1. front_scoop_callback(self, msg)
    2. back_scoop_callback(self, msg)

## Pump Callbacks

    1. pump_callback(self, msg, pump)

## Status Callbacks

    1. motor_status(self)
        listens to see if a motor is currently having an error
        if an error is detected it should then publish the current id of the error'd motor to the gui for troubleshooting
    2. motor_reboot(self, msg)
        

## Gimbal Callbacks

    1. gimbal_pos(self, msg)
        Should be getting the goal position from the briance that will then be translated into 
        a radian(will find the correct unit of measurement later) before moving the camera slowly to that position
    2. gimbal_zero(self)
        helper method to reset the camera to a front facing position

## Helper Functions

    1. bytes_to_twos_complement(byte_list)
    2. get_cur_pos(motor, type)