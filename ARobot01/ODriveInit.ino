void OdriveInit() {

      for (int axis = 0; axis < 2; ++axis) {
          Serial1 << "w axis" << axis << ".controller.config.vel_limit " << 70000.0f << '\n';
          Serial1 << "w axis" << axis << ".motor.config.current_lim " << 30.0f << '\n';

          requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive1.run_state(axis, requested_state, true);
    
          requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive1.run_state(axis, requested_state, true);
    
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive1.run_state(axis, requested_state, false); // don't wait

          delay(500);

          Serial1 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
          Serial1 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';


      }

      for (int axis = 0; axis < 2; ++axis) {
          Serial2 << "w axis" << axis << ".controller.config.vel_limit " << 70000.0f << '\n';
          Serial2 << "w axis" << axis << ".motor.config.current_lim " << 30.0f << '\n';

          requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive2.run_state(axis, requested_state, true);
    
          requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive2.run_state(axis, requested_state, true);
    
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive2.run_state(axis, requested_state, false); // don't wait 

          delay(500);

          Serial2 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
          Serial2 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';

      }
            
         
  
}




