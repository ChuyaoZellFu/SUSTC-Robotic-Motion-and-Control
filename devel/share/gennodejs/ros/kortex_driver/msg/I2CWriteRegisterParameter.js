// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let I2CData = require('./I2CData.js');

//-----------------------------------------------------------

class I2CWriteRegisterParameter {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.device = null;
      this.device_address = null;
      this.register_address = null;
      this.register_address_size = null;
      this.timeout = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('device')) {
        this.device = initObj.device
      }
      else {
        this.device = 0;
      }
      if (initObj.hasOwnProperty('device_address')) {
        this.device_address = initObj.device_address
      }
      else {
        this.device_address = 0;
      }
      if (initObj.hasOwnProperty('register_address')) {
        this.register_address = initObj.register_address
      }
      else {
        this.register_address = 0;
      }
      if (initObj.hasOwnProperty('register_address_size')) {
        this.register_address_size = initObj.register_address_size
      }
      else {
        this.register_address_size = 0;
      }
      if (initObj.hasOwnProperty('timeout')) {
        this.timeout = initObj.timeout
      }
      else {
        this.timeout = 0;
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = new I2CData();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type I2CWriteRegisterParameter
    // Serialize message field [device]
    bufferOffset = _serializer.uint32(obj.device, buffer, bufferOffset);
    // Serialize message field [device_address]
    bufferOffset = _serializer.uint32(obj.device_address, buffer, bufferOffset);
    // Serialize message field [register_address]
    bufferOffset = _serializer.uint32(obj.register_address, buffer, bufferOffset);
    // Serialize message field [register_address_size]
    bufferOffset = _serializer.uint32(obj.register_address_size, buffer, bufferOffset);
    // Serialize message field [timeout]
    bufferOffset = _serializer.uint32(obj.timeout, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = I2CData.serialize(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type I2CWriteRegisterParameter
    let len;
    let data = new I2CWriteRegisterParameter(null);
    // Deserialize message field [device]
    data.device = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [device_address]
    data.device_address = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [register_address]
    data.register_address = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [register_address_size]
    data.register_address_size = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [timeout]
    data.timeout = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = I2CData.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += I2CData.getMessageSize(object.data);
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/I2CWriteRegisterParameter';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '347168899c8760463d3fc21c4d811648';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 device
    uint32 device_address
    uint32 register_address
    uint32 register_address_size
    uint32 timeout
    I2CData data
    ================================================================================
    MSG: kortex_driver/I2CData
    
    uint8[] data
    uint32 size
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new I2CWriteRegisterParameter(null);
    if (msg.device !== undefined) {
      resolved.device = msg.device;
    }
    else {
      resolved.device = 0
    }

    if (msg.device_address !== undefined) {
      resolved.device_address = msg.device_address;
    }
    else {
      resolved.device_address = 0
    }

    if (msg.register_address !== undefined) {
      resolved.register_address = msg.register_address;
    }
    else {
      resolved.register_address = 0
    }

    if (msg.register_address_size !== undefined) {
      resolved.register_address_size = msg.register_address_size;
    }
    else {
      resolved.register_address_size = 0
    }

    if (msg.timeout !== undefined) {
      resolved.timeout = msg.timeout;
    }
    else {
      resolved.timeout = 0
    }

    if (msg.data !== undefined) {
      resolved.data = I2CData.Resolve(msg.data)
    }
    else {
      resolved.data = new I2CData()
    }

    return resolved;
    }
};

module.exports = I2CWriteRegisterParameter;
