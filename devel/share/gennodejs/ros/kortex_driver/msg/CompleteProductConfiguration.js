// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CountryCode = require('./CountryCode.js');

//-----------------------------------------------------------

class CompleteProductConfiguration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.kin = null;
      this.model = null;
      this.country_code = null;
      this.assembly_plant = null;
      this.model_year = null;
      this.degree_of_freedom = null;
      this.base_type = null;
      this.end_effector_type = null;
      this.vision_module_type = null;
      this.interface_module_type = null;
      this.arm_laterality = null;
      this.wrist_type = null;
      this.brake_type = null;
    }
    else {
      if (initObj.hasOwnProperty('kin')) {
        this.kin = initObj.kin
      }
      else {
        this.kin = '';
      }
      if (initObj.hasOwnProperty('model')) {
        this.model = initObj.model
      }
      else {
        this.model = 0;
      }
      if (initObj.hasOwnProperty('country_code')) {
        this.country_code = initObj.country_code
      }
      else {
        this.country_code = new CountryCode();
      }
      if (initObj.hasOwnProperty('assembly_plant')) {
        this.assembly_plant = initObj.assembly_plant
      }
      else {
        this.assembly_plant = '';
      }
      if (initObj.hasOwnProperty('model_year')) {
        this.model_year = initObj.model_year
      }
      else {
        this.model_year = '';
      }
      if (initObj.hasOwnProperty('degree_of_freedom')) {
        this.degree_of_freedom = initObj.degree_of_freedom
      }
      else {
        this.degree_of_freedom = 0;
      }
      if (initObj.hasOwnProperty('base_type')) {
        this.base_type = initObj.base_type
      }
      else {
        this.base_type = 0;
      }
      if (initObj.hasOwnProperty('end_effector_type')) {
        this.end_effector_type = initObj.end_effector_type
      }
      else {
        this.end_effector_type = 0;
      }
      if (initObj.hasOwnProperty('vision_module_type')) {
        this.vision_module_type = initObj.vision_module_type
      }
      else {
        this.vision_module_type = 0;
      }
      if (initObj.hasOwnProperty('interface_module_type')) {
        this.interface_module_type = initObj.interface_module_type
      }
      else {
        this.interface_module_type = 0;
      }
      if (initObj.hasOwnProperty('arm_laterality')) {
        this.arm_laterality = initObj.arm_laterality
      }
      else {
        this.arm_laterality = 0;
      }
      if (initObj.hasOwnProperty('wrist_type')) {
        this.wrist_type = initObj.wrist_type
      }
      else {
        this.wrist_type = 0;
      }
      if (initObj.hasOwnProperty('brake_type')) {
        this.brake_type = initObj.brake_type
      }
      else {
        this.brake_type = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CompleteProductConfiguration
    // Serialize message field [kin]
    bufferOffset = _serializer.string(obj.kin, buffer, bufferOffset);
    // Serialize message field [model]
    bufferOffset = _serializer.uint32(obj.model, buffer, bufferOffset);
    // Serialize message field [country_code]
    bufferOffset = CountryCode.serialize(obj.country_code, buffer, bufferOffset);
    // Serialize message field [assembly_plant]
    bufferOffset = _serializer.string(obj.assembly_plant, buffer, bufferOffset);
    // Serialize message field [model_year]
    bufferOffset = _serializer.string(obj.model_year, buffer, bufferOffset);
    // Serialize message field [degree_of_freedom]
    bufferOffset = _serializer.uint32(obj.degree_of_freedom, buffer, bufferOffset);
    // Serialize message field [base_type]
    bufferOffset = _serializer.uint32(obj.base_type, buffer, bufferOffset);
    // Serialize message field [end_effector_type]
    bufferOffset = _serializer.uint32(obj.end_effector_type, buffer, bufferOffset);
    // Serialize message field [vision_module_type]
    bufferOffset = _serializer.uint32(obj.vision_module_type, buffer, bufferOffset);
    // Serialize message field [interface_module_type]
    bufferOffset = _serializer.uint32(obj.interface_module_type, buffer, bufferOffset);
    // Serialize message field [arm_laterality]
    bufferOffset = _serializer.uint32(obj.arm_laterality, buffer, bufferOffset);
    // Serialize message field [wrist_type]
    bufferOffset = _serializer.uint32(obj.wrist_type, buffer, bufferOffset);
    // Serialize message field [brake_type]
    bufferOffset = _serializer.uint32(obj.brake_type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CompleteProductConfiguration
    let len;
    let data = new CompleteProductConfiguration(null);
    // Deserialize message field [kin]
    data.kin = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [model]
    data.model = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [country_code]
    data.country_code = CountryCode.deserialize(buffer, bufferOffset);
    // Deserialize message field [assembly_plant]
    data.assembly_plant = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [model_year]
    data.model_year = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [degree_of_freedom]
    data.degree_of_freedom = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [base_type]
    data.base_type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [end_effector_type]
    data.end_effector_type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [vision_module_type]
    data.vision_module_type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [interface_module_type]
    data.interface_module_type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [arm_laterality]
    data.arm_laterality = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [wrist_type]
    data.wrist_type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [brake_type]
    data.brake_type = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.kin);
    length += _getByteLength(object.assembly_plant);
    length += _getByteLength(object.model_year);
    return length + 52;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/CompleteProductConfiguration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b73bc25bff314a65c4c98c0970043ccf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    string kin
    uint32 model
    CountryCode country_code
    string assembly_plant
    string model_year
    uint32 degree_of_freedom
    uint32 base_type
    uint32 end_effector_type
    uint32 vision_module_type
    uint32 interface_module_type
    uint32 arm_laterality
    uint32 wrist_type
    uint32 brake_type
    ================================================================================
    MSG: kortex_driver/CountryCode
    
    uint32 identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CompleteProductConfiguration(null);
    if (msg.kin !== undefined) {
      resolved.kin = msg.kin;
    }
    else {
      resolved.kin = ''
    }

    if (msg.model !== undefined) {
      resolved.model = msg.model;
    }
    else {
      resolved.model = 0
    }

    if (msg.country_code !== undefined) {
      resolved.country_code = CountryCode.Resolve(msg.country_code)
    }
    else {
      resolved.country_code = new CountryCode()
    }

    if (msg.assembly_plant !== undefined) {
      resolved.assembly_plant = msg.assembly_plant;
    }
    else {
      resolved.assembly_plant = ''
    }

    if (msg.model_year !== undefined) {
      resolved.model_year = msg.model_year;
    }
    else {
      resolved.model_year = ''
    }

    if (msg.degree_of_freedom !== undefined) {
      resolved.degree_of_freedom = msg.degree_of_freedom;
    }
    else {
      resolved.degree_of_freedom = 0
    }

    if (msg.base_type !== undefined) {
      resolved.base_type = msg.base_type;
    }
    else {
      resolved.base_type = 0
    }

    if (msg.end_effector_type !== undefined) {
      resolved.end_effector_type = msg.end_effector_type;
    }
    else {
      resolved.end_effector_type = 0
    }

    if (msg.vision_module_type !== undefined) {
      resolved.vision_module_type = msg.vision_module_type;
    }
    else {
      resolved.vision_module_type = 0
    }

    if (msg.interface_module_type !== undefined) {
      resolved.interface_module_type = msg.interface_module_type;
    }
    else {
      resolved.interface_module_type = 0
    }

    if (msg.arm_laterality !== undefined) {
      resolved.arm_laterality = msg.arm_laterality;
    }
    else {
      resolved.arm_laterality = 0
    }

    if (msg.wrist_type !== undefined) {
      resolved.wrist_type = msg.wrist_type;
    }
    else {
      resolved.wrist_type = 0
    }

    if (msg.brake_type !== undefined) {
      resolved.brake_type = msg.brake_type;
    }
    else {
      resolved.brake_type = 0
    }

    return resolved;
    }
};

module.exports = CompleteProductConfiguration;
