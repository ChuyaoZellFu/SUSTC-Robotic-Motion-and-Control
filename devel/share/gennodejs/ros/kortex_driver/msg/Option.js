// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Option {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Option
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Option
    let len;
    let data = new Option(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Option';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0f091628538d1f237d2c9bbe103cea73';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 OPTION_UNSPECIFIED = 0
    
    uint32 OPTION_BACKLIGHT_COMPENSATION = 1
    
    uint32 OPTION_BRIGHTNESS = 2
    
    uint32 OPTION_CONTRAST = 3
    
    uint32 OPTION_EXPOSURE = 4
    
    uint32 OPTION_GAIN = 5
    
    uint32 OPTION_GAMMA = 6
    
    uint32 OPTION_HUE = 7
    
    uint32 OPTION_SATURATION = 8
    
    uint32 OPTION_SHARPNESS = 9
    
    uint32 OPTION_WHITE_BALANCE = 10
    
    uint32 OPTION_ENABLE_AUTO_EXPOSURE = 11
    
    uint32 OPTION_ENABLE_AUTO_WHITE_BALANCE = 12
    
    uint32 OPTION_VISUAL_PRESET = 13
    
    uint32 OPTION_LASER_POWER = 14
    
    uint32 OPTION_ACCURACY = 15
    
    uint32 OPTION_MOTION_RANGE = 16
    
    uint32 OPTION_FILTER_OPTION = 17
    
    uint32 OPTION_CONFIDENCE_THRESHOLD = 18
    
    uint32 OPTION_EMITTER_ENABLED = 19
    
    uint32 OPTION_FRAMES_QUEUE_SIZE = 20
    
    uint32 OPTION_TOTAL_FRAME_DROPS = 21
    
    uint32 OPTION_AUTO_EXPOSURE_MODE = 22
    
    uint32 OPTION_POWER_LINE_FREQUENCY = 23
    
    uint32 OPTION_ASIC_TEMPERATURE = 24
    
    uint32 OPTION_ERROR_POLLING_ENABLED = 25
    
    uint32 OPTION_PROJECTOR_TEMPERATURE = 26
    
    uint32 OPTION_OUTPUT_TRIGGER_ENABLED = 27
    
    uint32 OPTION_MOTION_MODULE_TEMPERATURE = 28
    
    uint32 OPTION_DEPTH_UNITS = 29
    
    uint32 OPTION_ENABLE_MOTION_CORRECTION = 30
    
    uint32 OPTION_AUTO_EXPOSURE_PRIORITY = 31
    
    uint32 OPTION_COLOR_SCHEME = 32
    
    uint32 OPTION_HISTOGRAM_EQUALIZATION_ENABLED = 33
    
    uint32 OPTION_MIN_DISTANCE = 34
    
    uint32 OPTION_MAX_DISTANCE = 35
    
    uint32 OPTION_TEXTURE_SOURCE = 36
    
    uint32 OPTION_FILTER_MAGNITUDE = 37
    
    uint32 OPTION_FILTER_SMOOTH_ALPHA = 38
    
    uint32 OPTION_FILTER_SMOOTH_DELTA = 39
    
    uint32 OPTION_HOLES_FILL = 40
    
    uint32 OPTION_STEREO_BASELINE = 41
    
    uint32 OPTION_AUTO_EXPOSURE_CONVERGE_STEP = 42
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Option(null);
    return resolved;
    }
};

// Constants for message
Option.Constants = {
  OPTION_UNSPECIFIED: 0,
  OPTION_BACKLIGHT_COMPENSATION: 1,
  OPTION_BRIGHTNESS: 2,
  OPTION_CONTRAST: 3,
  OPTION_EXPOSURE: 4,
  OPTION_GAIN: 5,
  OPTION_GAMMA: 6,
  OPTION_HUE: 7,
  OPTION_SATURATION: 8,
  OPTION_SHARPNESS: 9,
  OPTION_WHITE_BALANCE: 10,
  OPTION_ENABLE_AUTO_EXPOSURE: 11,
  OPTION_ENABLE_AUTO_WHITE_BALANCE: 12,
  OPTION_VISUAL_PRESET: 13,
  OPTION_LASER_POWER: 14,
  OPTION_ACCURACY: 15,
  OPTION_MOTION_RANGE: 16,
  OPTION_FILTER_OPTION: 17,
  OPTION_CONFIDENCE_THRESHOLD: 18,
  OPTION_EMITTER_ENABLED: 19,
  OPTION_FRAMES_QUEUE_SIZE: 20,
  OPTION_TOTAL_FRAME_DROPS: 21,
  OPTION_AUTO_EXPOSURE_MODE: 22,
  OPTION_POWER_LINE_FREQUENCY: 23,
  OPTION_ASIC_TEMPERATURE: 24,
  OPTION_ERROR_POLLING_ENABLED: 25,
  OPTION_PROJECTOR_TEMPERATURE: 26,
  OPTION_OUTPUT_TRIGGER_ENABLED: 27,
  OPTION_MOTION_MODULE_TEMPERATURE: 28,
  OPTION_DEPTH_UNITS: 29,
  OPTION_ENABLE_MOTION_CORRECTION: 30,
  OPTION_AUTO_EXPOSURE_PRIORITY: 31,
  OPTION_COLOR_SCHEME: 32,
  OPTION_HISTOGRAM_EQUALIZATION_ENABLED: 33,
  OPTION_MIN_DISTANCE: 34,
  OPTION_MAX_DISTANCE: 35,
  OPTION_TEXTURE_SOURCE: 36,
  OPTION_FILTER_MAGNITUDE: 37,
  OPTION_FILTER_SMOOTH_ALPHA: 38,
  OPTION_FILTER_SMOOTH_DELTA: 39,
  OPTION_HOLES_FILL: 40,
  OPTION_STEREO_BASELINE: 41,
  OPTION_AUTO_EXPOSURE_CONVERGE_STEP: 42,
}

module.exports = Option;
