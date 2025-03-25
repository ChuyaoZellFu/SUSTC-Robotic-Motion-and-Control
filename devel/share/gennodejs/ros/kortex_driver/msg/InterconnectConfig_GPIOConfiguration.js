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

class InterconnectConfig_GPIOConfiguration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.identifier = null;
      this.mode = null;
      this.pull = null;
      this.default_value = null;
    }
    else {
      if (initObj.hasOwnProperty('identifier')) {
        this.identifier = initObj.identifier
      }
      else {
        this.identifier = 0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('pull')) {
        this.pull = initObj.pull
      }
      else {
        this.pull = 0;
      }
      if (initObj.hasOwnProperty('default_value')) {
        this.default_value = initObj.default_value
      }
      else {
        this.default_value = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InterconnectConfig_GPIOConfiguration
    // Serialize message field [identifier]
    bufferOffset = _serializer.uint32(obj.identifier, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint32(obj.mode, buffer, bufferOffset);
    // Serialize message field [pull]
    bufferOffset = _serializer.uint32(obj.pull, buffer, bufferOffset);
    // Serialize message field [default_value]
    bufferOffset = _serializer.uint32(obj.default_value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InterconnectConfig_GPIOConfiguration
    let len;
    let data = new InterconnectConfig_GPIOConfiguration(null);
    // Deserialize message field [identifier]
    data.identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [pull]
    data.pull = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [default_value]
    data.default_value = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/InterconnectConfig_GPIOConfiguration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'beac18d5b1b9bb04dc5a1a2c0d366dac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 identifier
    uint32 mode
    uint32 pull
    uint32 default_value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InterconnectConfig_GPIOConfiguration(null);
    if (msg.identifier !== undefined) {
      resolved.identifier = msg.identifier;
    }
    else {
      resolved.identifier = 0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.pull !== undefined) {
      resolved.pull = msg.pull;
    }
    else {
      resolved.pull = 0
    }

    if (msg.default_value !== undefined) {
      resolved.default_value = msg.default_value;
    }
    else {
      resolved.default_value = 0
    }

    return resolved;
    }
};

module.exports = InterconnectConfig_GPIOConfiguration;
