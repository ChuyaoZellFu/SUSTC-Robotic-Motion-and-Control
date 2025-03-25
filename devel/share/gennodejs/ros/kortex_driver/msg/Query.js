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

//-----------------------------------------------------------

class Query {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start_timestamp = null;
      this.end_timestamp = null;
      this.username = null;
    }
    else {
      if (initObj.hasOwnProperty('start_timestamp')) {
        this.start_timestamp = initObj.start_timestamp
      }
      else {
        this.start_timestamp = new Timestamp();
      }
      if (initObj.hasOwnProperty('end_timestamp')) {
        this.end_timestamp = initObj.end_timestamp
      }
      else {
        this.end_timestamp = new Timestamp();
      }
      if (initObj.hasOwnProperty('username')) {
        this.username = initObj.username
      }
      else {
        this.username = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Query
    // Serialize message field [start_timestamp]
    bufferOffset = Timestamp.serialize(obj.start_timestamp, buffer, bufferOffset);
    // Serialize message field [end_timestamp]
    bufferOffset = Timestamp.serialize(obj.end_timestamp, buffer, bufferOffset);
    // Serialize message field [username]
    bufferOffset = _serializer.string(obj.username, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Query
    let len;
    let data = new Query(null);
    // Deserialize message field [start_timestamp]
    data.start_timestamp = Timestamp.deserialize(buffer, bufferOffset);
    // Deserialize message field [end_timestamp]
    data.end_timestamp = Timestamp.deserialize(buffer, bufferOffset);
    // Deserialize message field [username]
    data.username = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.username);
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Query';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '13ee5e330f26cd5fd95364d08fb2a910';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    Timestamp start_timestamp
    Timestamp end_timestamp
    string username
    ================================================================================
    MSG: kortex_driver/Timestamp
    
    uint32 sec
    uint32 usec
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Query(null);
    if (msg.start_timestamp !== undefined) {
      resolved.start_timestamp = Timestamp.Resolve(msg.start_timestamp)
    }
    else {
      resolved.start_timestamp = new Timestamp()
    }

    if (msg.end_timestamp !== undefined) {
      resolved.end_timestamp = Timestamp.Resolve(msg.end_timestamp)
    }
    else {
      resolved.end_timestamp = new Timestamp()
    }

    if (msg.username !== undefined) {
      resolved.username = msg.username;
    }
    else {
      resolved.username = ''
    }

    return resolved;
    }
};

module.exports = Query;
