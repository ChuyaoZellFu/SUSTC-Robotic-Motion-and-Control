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

class PartNumberRevision {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.part_number_revision = null;
    }
    else {
      if (initObj.hasOwnProperty('part_number_revision')) {
        this.part_number_revision = initObj.part_number_revision
      }
      else {
        this.part_number_revision = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PartNumberRevision
    // Serialize message field [part_number_revision]
    bufferOffset = _serializer.string(obj.part_number_revision, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PartNumberRevision
    let len;
    let data = new PartNumberRevision(null);
    // Deserialize message field [part_number_revision]
    data.part_number_revision = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.part_number_revision);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/PartNumberRevision';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4621433c3d6219edd88cd50a0ae9aa72';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    string part_number_revision
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PartNumberRevision(null);
    if (msg.part_number_revision !== undefined) {
      resolved.part_number_revision = msg.part_number_revision;
    }
    else {
      resolved.part_number_revision = ''
    }

    return resolved;
    }
};

module.exports = PartNumberRevision;
