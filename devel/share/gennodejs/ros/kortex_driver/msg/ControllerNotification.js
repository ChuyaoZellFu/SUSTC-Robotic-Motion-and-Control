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
let ControllerNotification_state = require('./ControllerNotification_state.js');

//-----------------------------------------------------------

class ControllerNotification {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timestamp = null;
      this.user_handle = null;
      this.connection = null;
      this.oneof_state = null;
    }
    else {
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
      if (initObj.hasOwnProperty('oneof_state')) {
        this.oneof_state = initObj.oneof_state
      }
      else {
        this.oneof_state = new ControllerNotification_state();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerNotification
    // Serialize message field [timestamp]
    bufferOffset = Timestamp.serialize(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [user_handle]
    bufferOffset = UserProfileHandle.serialize(obj.user_handle, buffer, bufferOffset);
    // Serialize message field [connection]
    bufferOffset = Connection.serialize(obj.connection, buffer, bufferOffset);
    // Serialize message field [oneof_state]
    bufferOffset = ControllerNotification_state.serialize(obj.oneof_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerNotification
    let len;
    let data = new ControllerNotification(null);
    // Deserialize message field [timestamp]
    data.timestamp = Timestamp.deserialize(buffer, bufferOffset);
    // Deserialize message field [user_handle]
    data.user_handle = UserProfileHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [connection]
    data.connection = Connection.deserialize(buffer, bufferOffset);
    // Deserialize message field [oneof_state]
    data.oneof_state = ControllerNotification_state.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Connection.getMessageSize(object.connection);
    length += ControllerNotification_state.getMessageSize(object.oneof_state);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControllerNotification';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4a8a872ea40ec244a141a390ef3880ea';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    Timestamp timestamp
    UserProfileHandle user_handle
    Connection connection
    ControllerNotification_state oneof_state
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
    MSG: kortex_driver/ControllerNotification_state
    
    ControllerState[] controller_state
    ControllerElementState[] controller_element
    ================================================================================
    MSG: kortex_driver/ControllerState
    
    ControllerHandle handle
    uint32 event_type
    ================================================================================
    MSG: kortex_driver/ControllerHandle
    
    uint32 type
    uint32 controller_identifier
    ================================================================================
    MSG: kortex_driver/ControllerElementState
    
    ControllerElementHandle handle
    uint32 event_type
    float32 axis_value
    ================================================================================
    MSG: kortex_driver/ControllerElementHandle
    
    ControllerHandle controller_handle
    ControllerElementHandle_identifier oneof_identifier
    ================================================================================
    MSG: kortex_driver/ControllerElementHandle_identifier
    
    uint32[] button
    uint32[] axis
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerNotification(null);
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

    if (msg.oneof_state !== undefined) {
      resolved.oneof_state = ControllerNotification_state.Resolve(msg.oneof_state)
    }
    else {
      resolved.oneof_state = new ControllerNotification_state()
    }

    return resolved;
    }
};

module.exports = ControllerNotification;
