// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ProtectionZoneHandle = require('./ProtectionZoneHandle.js');
let Timestamp = require('./Timestamp.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let Connection = require('./Connection.js');

//-----------------------------------------------------------

class ProtectionZoneNotification {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.event = null;
      this.handle = null;
      this.timestamp = null;
      this.user_handle = null;
      this.connection = null;
    }
    else {
      if (initObj.hasOwnProperty('event')) {
        this.event = initObj.event
      }
      else {
        this.event = 0;
      }
      if (initObj.hasOwnProperty('handle')) {
        this.handle = initObj.handle
      }
      else {
        this.handle = new ProtectionZoneHandle();
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ProtectionZoneNotification
    // Serialize message field [event]
    bufferOffset = _serializer.uint32(obj.event, buffer, bufferOffset);
    // Serialize message field [handle]
    bufferOffset = ProtectionZoneHandle.serialize(obj.handle, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = Timestamp.serialize(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [user_handle]
    bufferOffset = UserProfileHandle.serialize(obj.user_handle, buffer, bufferOffset);
    // Serialize message field [connection]
    bufferOffset = Connection.serialize(obj.connection, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ProtectionZoneNotification
    let len;
    let data = new ProtectionZoneNotification(null);
    // Deserialize message field [event]
    data.event = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [handle]
    data.handle = ProtectionZoneHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = Timestamp.deserialize(buffer, bufferOffset);
    // Deserialize message field [user_handle]
    data.user_handle = UserProfileHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [connection]
    data.connection = Connection.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Connection.getMessageSize(object.connection);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ProtectionZoneNotification';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2ba41a2c5a55d12e33dfcd059ebe2676';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 event
    ProtectionZoneHandle handle
    Timestamp timestamp
    UserProfileHandle user_handle
    Connection connection
    ================================================================================
    MSG: kortex_driver/ProtectionZoneHandle
    
    uint32 identifier
    uint32 permission
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ProtectionZoneNotification(null);
    if (msg.event !== undefined) {
      resolved.event = msg.event;
    }
    else {
      resolved.event = 0
    }

    if (msg.handle !== undefined) {
      resolved.handle = ProtectionZoneHandle.Resolve(msg.handle)
    }
    else {
      resolved.handle = new ProtectionZoneHandle()
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

    return resolved;
    }
};

module.exports = ProtectionZoneNotification;
