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

class UARTDeviceIdentification {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.port_id = null;
    }
    else {
      if (initObj.hasOwnProperty('port_id')) {
        this.port_id = initObj.port_id
      }
      else {
        this.port_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UARTDeviceIdentification
    // Serialize message field [port_id]
    bufferOffset = _serializer.uint32(obj.port_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UARTDeviceIdentification
    let len;
    let data = new UARTDeviceIdentification(null);
    // Deserialize message field [port_id]
    data.port_id = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/UARTDeviceIdentification';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c688e9b638ff5d4cab989955c0b36eb8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 port_id
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UARTDeviceIdentification(null);
    if (msg.port_id !== undefined) {
      resolved.port_id = msg.port_id;
    }
    else {
      resolved.port_id = 0
    }

    return resolved;
    }
};

module.exports = UARTDeviceIdentification;
