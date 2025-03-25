// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let UserProfileHandle = require('./UserProfileHandle.js');

//-----------------------------------------------------------

class Connection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.user_handle = null;
      this.connection_information = null;
      this.connection_identifier = null;
    }
    else {
      if (initObj.hasOwnProperty('user_handle')) {
        this.user_handle = initObj.user_handle
      }
      else {
        this.user_handle = new UserProfileHandle();
      }
      if (initObj.hasOwnProperty('connection_information')) {
        this.connection_information = initObj.connection_information
      }
      else {
        this.connection_information = '';
      }
      if (initObj.hasOwnProperty('connection_identifier')) {
        this.connection_identifier = initObj.connection_identifier
      }
      else {
        this.connection_identifier = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Connection
    // Serialize message field [user_handle]
    bufferOffset = UserProfileHandle.serialize(obj.user_handle, buffer, bufferOffset);
    // Serialize message field [connection_information]
    bufferOffset = _serializer.string(obj.connection_information, buffer, bufferOffset);
    // Serialize message field [connection_identifier]
    bufferOffset = _serializer.uint32(obj.connection_identifier, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Connection
    let len;
    let data = new Connection(null);
    // Deserialize message field [user_handle]
    data.user_handle = UserProfileHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [connection_information]
    data.connection_information = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [connection_identifier]
    data.connection_identifier = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.connection_information);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Connection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '149818e986cd66f3e32843abf7b3bfe5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    UserProfileHandle user_handle
    string connection_information
    uint32 connection_identifier
    ================================================================================
    MSG: kortex_driver/UserProfileHandle
    
    uint32 identifier
    uint32 permission
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Connection(null);
    if (msg.user_handle !== undefined) {
      resolved.user_handle = UserProfileHandle.Resolve(msg.user_handle)
    }
    else {
      resolved.user_handle = new UserProfileHandle()
    }

    if (msg.connection_information !== undefined) {
      resolved.connection_information = msg.connection_information;
    }
    else {
      resolved.connection_information = ''
    }

    if (msg.connection_identifier !== undefined) {
      resolved.connection_identifier = msg.connection_identifier;
    }
    else {
      resolved.connection_identifier = 0
    }

    return resolved;
    }
};

module.exports = Connection;
