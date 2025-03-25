// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let VisionConfig_RotationMatrix = require('./VisionConfig_RotationMatrix.js');
let TranslationVector = require('./TranslationVector.js');

//-----------------------------------------------------------

class ExtrinsicParameters {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rotation = null;
      this.translation = null;
    }
    else {
      if (initObj.hasOwnProperty('rotation')) {
        this.rotation = initObj.rotation
      }
      else {
        this.rotation = new VisionConfig_RotationMatrix();
      }
      if (initObj.hasOwnProperty('translation')) {
        this.translation = initObj.translation
      }
      else {
        this.translation = new TranslationVector();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ExtrinsicParameters
    // Serialize message field [rotation]
    bufferOffset = VisionConfig_RotationMatrix.serialize(obj.rotation, buffer, bufferOffset);
    // Serialize message field [translation]
    bufferOffset = TranslationVector.serialize(obj.translation, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExtrinsicParameters
    let len;
    let data = new ExtrinsicParameters(null);
    // Deserialize message field [rotation]
    data.rotation = VisionConfig_RotationMatrix.deserialize(buffer, bufferOffset);
    // Deserialize message field [translation]
    data.translation = TranslationVector.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ExtrinsicParameters';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '931ae3c2dcff1b69750ed6505e17f9d8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    VisionConfig_RotationMatrix rotation
    TranslationVector translation
    ================================================================================
    MSG: kortex_driver/VisionConfig_RotationMatrix
    
    VisionConfig_RotationMatrixRow row1
    VisionConfig_RotationMatrixRow row2
    VisionConfig_RotationMatrixRow row3
    ================================================================================
    MSG: kortex_driver/VisionConfig_RotationMatrixRow
    
    float32 column1
    float32 column2
    float32 column3
    ================================================================================
    MSG: kortex_driver/TranslationVector
    
    float32 t_x
    float32 t_y
    float32 t_z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ExtrinsicParameters(null);
    if (msg.rotation !== undefined) {
      resolved.rotation = VisionConfig_RotationMatrix.Resolve(msg.rotation)
    }
    else {
      resolved.rotation = new VisionConfig_RotationMatrix()
    }

    if (msg.translation !== undefined) {
      resolved.translation = TranslationVector.Resolve(msg.translation)
    }
    else {
      resolved.translation = new TranslationVector()
    }

    return resolved;
    }
};

module.exports = ExtrinsicParameters;
