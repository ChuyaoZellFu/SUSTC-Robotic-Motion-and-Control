// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Point = require('./Point.js');
let Base_RotationMatrix = require('./Base_RotationMatrix.js');

//-----------------------------------------------------------

class ZoneShape {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.shape_type = null;
      this.origin = null;
      this.orientation = null;
      this.dimensions = null;
      this.envelope_thickness = null;
    }
    else {
      if (initObj.hasOwnProperty('shape_type')) {
        this.shape_type = initObj.shape_type
      }
      else {
        this.shape_type = 0;
      }
      if (initObj.hasOwnProperty('origin')) {
        this.origin = initObj.origin
      }
      else {
        this.origin = new Point();
      }
      if (initObj.hasOwnProperty('orientation')) {
        this.orientation = initObj.orientation
      }
      else {
        this.orientation = new Base_RotationMatrix();
      }
      if (initObj.hasOwnProperty('dimensions')) {
        this.dimensions = initObj.dimensions
      }
      else {
        this.dimensions = [];
      }
      if (initObj.hasOwnProperty('envelope_thickness')) {
        this.envelope_thickness = initObj.envelope_thickness
      }
      else {
        this.envelope_thickness = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ZoneShape
    // Serialize message field [shape_type]
    bufferOffset = _serializer.uint32(obj.shape_type, buffer, bufferOffset);
    // Serialize message field [origin]
    bufferOffset = Point.serialize(obj.origin, buffer, bufferOffset);
    // Serialize message field [orientation]
    bufferOffset = Base_RotationMatrix.serialize(obj.orientation, buffer, bufferOffset);
    // Serialize message field [dimensions]
    bufferOffset = _arraySerializer.float32(obj.dimensions, buffer, bufferOffset, null);
    // Serialize message field [envelope_thickness]
    bufferOffset = _serializer.float32(obj.envelope_thickness, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ZoneShape
    let len;
    let data = new ZoneShape(null);
    // Deserialize message field [shape_type]
    data.shape_type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [origin]
    data.origin = Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [orientation]
    data.orientation = Base_RotationMatrix.deserialize(buffer, bufferOffset);
    // Deserialize message field [dimensions]
    data.dimensions = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [envelope_thickness]
    data.envelope_thickness = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.dimensions.length;
    return length + 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ZoneShape';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8de31189535cb06cb8caf5fc49d6c1b4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 shape_type
    Point origin
    Base_RotationMatrix orientation
    float32[] dimensions
    float32 envelope_thickness
    ================================================================================
    MSG: kortex_driver/Point
    
    float32 x
    float32 y
    float32 z
    ================================================================================
    MSG: kortex_driver/Base_RotationMatrix
    
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
    const resolved = new ZoneShape(null);
    if (msg.shape_type !== undefined) {
      resolved.shape_type = msg.shape_type;
    }
    else {
      resolved.shape_type = 0
    }

    if (msg.origin !== undefined) {
      resolved.origin = Point.Resolve(msg.origin)
    }
    else {
      resolved.origin = new Point()
    }

    if (msg.orientation !== undefined) {
      resolved.orientation = Base_RotationMatrix.Resolve(msg.orientation)
    }
    else {
      resolved.orientation = new Base_RotationMatrix()
    }

    if (msg.dimensions !== undefined) {
      resolved.dimensions = msg.dimensions;
    }
    else {
      resolved.dimensions = []
    }

    if (msg.envelope_thickness !== undefined) {
      resolved.envelope_thickness = msg.envelope_thickness;
    }
    else {
      resolved.envelope_thickness = 0.0
    }

    return resolved;
    }
};

module.exports = ZoneShape;
