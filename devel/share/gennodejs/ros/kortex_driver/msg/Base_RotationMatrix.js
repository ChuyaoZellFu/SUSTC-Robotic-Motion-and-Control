// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Base_RotationMatrixRow = require('./Base_RotationMatrixRow.js');

//-----------------------------------------------------------

class Base_RotationMatrix {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.row1 = null;
      this.row2 = null;
      this.row3 = null;
    }
    else {
      if (initObj.hasOwnProperty('row1')) {
        this.row1 = initObj.row1
      }
      else {
        this.row1 = new Base_RotationMatrixRow();
      }
      if (initObj.hasOwnProperty('row2')) {
        this.row2 = initObj.row2
      }
      else {
        this.row2 = new Base_RotationMatrixRow();
      }
      if (initObj.hasOwnProperty('row3')) {
        this.row3 = initObj.row3
      }
      else {
        this.row3 = new Base_RotationMatrixRow();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Base_RotationMatrix
    // Serialize message field [row1]
    bufferOffset = Base_RotationMatrixRow.serialize(obj.row1, buffer, bufferOffset);
    // Serialize message field [row2]
    bufferOffset = Base_RotationMatrixRow.serialize(obj.row2, buffer, bufferOffset);
    // Serialize message field [row3]
    bufferOffset = Base_RotationMatrixRow.serialize(obj.row3, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Base_RotationMatrix
    let len;
    let data = new Base_RotationMatrix(null);
    // Deserialize message field [row1]
    data.row1 = Base_RotationMatrixRow.deserialize(buffer, bufferOffset);
    // Deserialize message field [row2]
    data.row2 = Base_RotationMatrixRow.deserialize(buffer, bufferOffset);
    // Deserialize message field [row3]
    data.row3 = Base_RotationMatrixRow.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Base_RotationMatrix';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '37bab3875ca8eda43ce39de02fa3b72b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    Base_RotationMatrixRow row1
    Base_RotationMatrixRow row2
    Base_RotationMatrixRow row3
    ================================================================================
    MSG: kortex_driver/Base_RotationMatrixRow
    
    float32 column1
    float32 column2
    float32 column3
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Base_RotationMatrix(null);
    if (msg.row1 !== undefined) {
      resolved.row1 = Base_RotationMatrixRow.Resolve(msg.row1)
    }
    else {
      resolved.row1 = new Base_RotationMatrixRow()
    }

    if (msg.row2 !== undefined) {
      resolved.row2 = Base_RotationMatrixRow.Resolve(msg.row2)
    }
    else {
      resolved.row2 = new Base_RotationMatrixRow()
    }

    if (msg.row3 !== undefined) {
      resolved.row3 = Base_RotationMatrixRow.Resolve(msg.row3)
    }
    else {
      resolved.row3 = new Base_RotationMatrixRow()
    }

    return resolved;
    }
};

module.exports = Base_RotationMatrix;
