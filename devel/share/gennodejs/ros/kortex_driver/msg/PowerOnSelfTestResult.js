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

class PowerOnSelfTestResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.power_on_self_test_result = null;
    }
    else {
      if (initObj.hasOwnProperty('power_on_self_test_result')) {
        this.power_on_self_test_result = initObj.power_on_self_test_result
      }
      else {
        this.power_on_self_test_result = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PowerOnSelfTestResult
    // Serialize message field [power_on_self_test_result]
    bufferOffset = _serializer.uint32(obj.power_on_self_test_result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PowerOnSelfTestResult
    let len;
    let data = new PowerOnSelfTestResult(null);
    // Deserialize message field [power_on_self_test_result]
    data.power_on_self_test_result = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/PowerOnSelfTestResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '927efe338b454b142f376d1936670eb0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 power_on_self_test_result
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PowerOnSelfTestResult(null);
    if (msg.power_on_self_test_result !== undefined) {
      resolved.power_on_self_test_result = msg.power_on_self_test_result;
    }
    else {
      resolved.power_on_self_test_result = 0
    }

    return resolved;
    }
};

module.exports = PowerOnSelfTestResult;
