// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Timestamp = require('./Timestamp.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let Connection = require('./Connection.js');
let DeviceHandle = require('./DeviceHandle.js');

//-----------------------------------------------------------

class OperatingModeNotification {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.operating_mode = null;
      this.timestamp = null;
      this.user_handle = null;
      this.connection = null;
      this.device_handle = null;
    }
    else {
      if (initObj.hasOwnProperty('operating_mode')) {
        this.operating_mode = initObj.operating_mode
      }
      else {
        this.operating_mode = 0;
      }
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = new Timestamp();
      }
      if (initObj.hasOwnProperty('user_handle')) {
        this.user_handle = initObj.user_handle
      }
      else {
        this.user_handle = new UserProfileHandle();
      }
      if (initObj.hasOwnProperty('connection')) {
        this.connection = initObj.connection
      }
      else {
        this.connection = new Connection();
      }
      if (initObj.hasOwnProperty('device_handle')) {
        this.device_handle = initObj.device_handle
      }
      else {
        this.device_handle = new DeviceHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OperatingModeNotification
    // Serialize message field [operating_mode]
    bufferOffset = _serializer.uint32(obj.operating_mode, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = Timestamp.serialize(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [user_handle]
    bufferOffset = UserProfileHandle.serialize(obj.user_handle, buffer, bufferOffset);
    // Serialize message field [connection]
    bufferOffset = Connection.serialize(obj.connection, buffer, bufferOffset);
    // Serialize message field [device_handle]
    bufferOffset = DeviceHandle.serialize(obj.device_handle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OperatingModeNotification
    let len;
    let data = new OperatingModeNotification(null);
    // Deserialize message field [operating_mode]
    data.operating_mode = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = Timestamp.deserialize(buffer, bufferOffset);
    // Deserialize message field [user_handle]
    data.user_handle = UserProfileHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [connection]
    data.connection = Connection.deserialize(buffer, bufferOffset);
    // Deserialize message field [device_handle]
    data.device_handle = DeviceHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Connection.getMessageSize(object.connection);
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/OperatingModeNotification';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3307c63e67ebe8d3a49769532d9dee31';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 operating_mode
    Timestamp timestamp
    UserProfileHandle user_handle
    Connection connection
    DeviceHandle device_handle
    ================================================================================
    MSG: kortex_driver/Timestamp
    
    uint32 sec
    uint32 usec
    ================================================================================
    MSG: kortex_driver/UserProfileHandle
    
    uint32 identifier
    uint32 permission
    ================================================================================
    MSG: kortex_driver/Connection
    
    UserProfileHandle user_handle
    string connection_information
    uint32 connection_identifier
    ================================================================================
    MSG: kortex_driver/DeviceHandle
    
    uint32 device_type
    uint32 device_identifier
    uint32 order
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OperatingModeNotification(null);
    if (msg.operating_mode !== undefined) {
      resolved.operating_mode = msg.operating_mode;
    }
    else {
      resolved.operating_mode = 0
    }

    if (msg.timestamp !== undefined) {
      resolved.timestamp = Timestamp.Resolve(msg.timestamp)
    }
    else {
      resolved.timestamp = new Timestamp()
    }

    if (msg.user_handle !== undefined) {
      resolved.user_handle = UserProfileHandle.Resolve(msg.user_handle)
    }
    else {
      resolved.user_handle = new UserProfileHandle()
    }

    if (msg.connection !== undefined) {
      resolved.connection = Connection.Resolve(msg.connection)
    }
    else {
      resolved.connection = new Connection()
    }

    if (msg.device_handle !== undefined) {
      resolved.device_handle = DeviceHandle.Resolve(msg.device_handle)
    }
    else {
      resolved.device_handle = new DeviceHandle()
    }

    return resolved;
    }
};

module.exports = OperatingModeNotification;
