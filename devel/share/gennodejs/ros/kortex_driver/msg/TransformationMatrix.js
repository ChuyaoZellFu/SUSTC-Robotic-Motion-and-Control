// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TransformationRow = require('./TransformationRow.js');

//-----------------------------------------------------------

class TransformationMatrix {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.r0 = null;
      this.r1 = null;
      this.r2 = null;
      this.r3 = null;
    }
    else {
      if (initObj.hasOwnProperty('r0')) {
        this.r0 = initObj.r0
      }
      else {
        this.r0 = new TransformationRow();
      }
      if (initObj.hasOwnProperty('r1')) {
        this.r1 = initObj.r1
      }
      else {
        this.r1 = new TransformationRow();
      }
      if (initObj.hasOwnProperty('r2')) {
        this.r2 = initObj.r2
      }
      else {
        this.r2 = new TransformationRow();
      }
      if (initObj.hasOwnProperty('r3')) {
        this.r3 = initObj.r3
      }
      else {
        this.r3 = new TransformationRow();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TransformationMatrix
    // Serialize message field [r0]
    bufferOffset = TransformationRow.serialize(obj.r0, buffer, bufferOffset);
    // Serialize message field [r1]
    bufferOffset = TransformationRow.serialize(obj.r1, buffer, bufferOffset);
    // Serialize message field [r2]
    bufferOffset = TransformationRow.serialize(obj.r2, buffer, bufferOffset);
    // Serialize message field [r3]
    bufferOffset = TransformationRow.serialize(obj.r3, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TransformationMatrix
    let len;
    let data = new TransformationMatrix(null);
    // Deserialize message field [r0]
    data.r0 = TransformationRow.deserialize(buffer, bufferOffset);
    // Deserialize message field [r1]
    data.r1 = TransformationRow.deserialize(buffer, bufferOffset);
    // Deserialize message field [r2]
    data.r2 = TransformationRow.deserialize(buffer, bufferOffset);
    // Deserialize message field [r3]
    data.r3 = TransformationRow.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/TransformationMatrix';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '01b579348ba31feab01058d980795bad';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    TransformationRow r0
    TransformationRow r1
    TransformationRow r2
    TransformationRow r3
    ================================================================================
    MSG: kortex_driver/TransformationRow
    
    float32 c0
    float32 c1
    float32 c2
    float32 c3
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TransformationMatrix(null);
    if (msg.r0 !== undefined) {
      resolved.r0 = TransformationRow.Resolve(msg.r0)
    }
    else {
      resolved.r0 = new TransformationRow()
    }

    if (msg.r1 !== undefined) {
      resolved.r1 = TransformationRow.Resolve(msg.r1)
    }
    else {
      resolved.r1 = new TransformationRow()
    }

    if (msg.r2 !== undefined) {
      resolved.r2 = TransformationRow.Resolve(msg.r2)
    }
    else {
      resolved.r2 = new TransformationRow()
    }

    if (msg.r3 !== undefined) {
      resolved.r3 = TransformationRow.Resolve(msg.r3)
    }
    else {
      resolved.r3 = new TransformationRow()
    }

    return resolved;
    }
};

module.exports = TransformationMatrix;
