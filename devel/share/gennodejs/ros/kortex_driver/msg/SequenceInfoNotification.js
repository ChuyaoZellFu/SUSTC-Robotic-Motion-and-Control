// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SequenceHandle = require('./SequenceHandle.js');
let Timestamp = require('./Timestamp.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let Connection = require('./Connection.js');

//-----------------------------------------------------------

class SequenceInfoNotification {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.event_identifier = null;
      this.sequence_handle = null;
      this.task_index = null;
      this.group_identifier = null;
      this.timestamp = null;
      this.user_handle = null;
      this.abort_details = null;
      this.connection = null;
    }
    else {
      if (initObj.hasOwnProperty('event_identifier')) {
        this.event_identifier = initObj.event_identifier
      }
      else {
        this.event_identifier = 0;
      }
      if (initObj.hasOwnProperty('sequence_handle')) {
        this.sequence_handle = initObj.sequence_handle
      }
      else {
        this.sequence_handle = new SequenceHandle();
      }
      if (initObj.hasOwnProperty('task_index')) {
        this.task_index = initObj.task_index
      }
      else {
        this.task_index = 0;
      }
      if (initObj.hasOwnProperty('group_identifier')) {
        this.group_identifier = initObj.group_identifier
      }
      else {
        this.group_identifier = 0;
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
      if (initObj.hasOwnProperty('abort_details')) {
        this.abort_details = initObj.abort_details
      }
      else {
        this.abort_details = 0;
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
    // Serializes a message object of type SequenceInfoNotification
    // Serialize message field [event_identifier]
    bufferOffset = _serializer.uint32(obj.event_identifier, buffer, bufferOffset);
    // Serialize message field [sequence_handle]
    bufferOffset = SequenceHandle.serialize(obj.sequence_handle, buffer, bufferOffset);
    // Serialize message field [task_index]
    bufferOffset = _serializer.uint32(obj.task_index, buffer, bufferOffset);
    // Serialize message field [group_identifier]
    bufferOffset = _serializer.uint32(obj.group_identifier, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = Timestamp.serialize(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [user_handle]
    bufferOffset = UserProfileHandle.serialize(obj.user_handle, buffer, bufferOffset);
    // Serialize message field [abort_details]
    bufferOffset = _serializer.uint32(obj.abort_details, buffer, bufferOffset);
    // Serialize message field [connection]
    bufferOffset = Connection.serialize(obj.connection, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SequenceInfoNotification
    let len;
    let data = new SequenceInfoNotification(null);
    // Deserialize message field [event_identifier]
    data.event_identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [sequence_handle]
    data.sequence_handle = SequenceHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [task_index]
    data.task_index = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [group_identifier]
    data.group_identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = Timestamp.deserialize(buffer, bufferOffset);
    // Deserialize message field [user_handle]
    data.user_handle = UserProfileHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [abort_details]
    data.abort_details = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [connection]
    data.connection = Connection.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Connection.getMessageSize(object.connection);
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SequenceInfoNotification';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cfd9f46ae5ef4976e20a716c9bac5aef';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 event_identifier
    SequenceHandle sequence_handle
    uint32 task_index
    uint32 group_identifier
    Timestamp timestamp
    UserProfileHandle user_handle
    uint32 abort_details
    Connection connection
    ================================================================================
    MSG: kortex_driver/SequenceHandle
    
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
    const resolved = new SequenceInfoNotification(null);
    if (msg.event_identifier !== undefined) {
      resolved.event_identifier = msg.event_identifier;
    }
    else {
      resolved.event_identifier = 0
    }

    if (msg.sequence_handle !== undefined) {
      resolved.sequence_handle = SequenceHandle.Resolve(msg.sequence_handle)
    }
    else {
      resolved.sequence_handle = new SequenceHandle()
    }

    if (msg.task_index !== undefined) {
      resolved.task_index = msg.task_index;
    }
    else {
      resolved.task_index = 0
    }

    if (msg.group_identifier !== undefined) {
      resolved.group_identifier = msg.group_identifier;
    }
    else {
      resolved.group_identifier = 0
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

    if (msg.abort_details !== undefined) {
      resolved.abort_details = msg.abort_details;
    }
    else {
      resolved.abort_details = 0
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

module.exports = SequenceInfoNotification;
