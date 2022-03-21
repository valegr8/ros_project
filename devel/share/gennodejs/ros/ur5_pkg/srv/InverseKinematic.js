// Auto-generated. Do not edit!

// (in-package ur5_pkg.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class InverseKinematicRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.z = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InverseKinematicRequest
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InverseKinematicRequest
    let len;
    let data = new InverseKinematicRequest(null);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur5_pkg/InverseKinematicRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4a842b65f413084dc2b10fb484ea7f17';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InverseKinematicRequest(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    return resolved;
    }
};

class InverseKinematicResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.theta0 = null;
      this.theta1 = null;
      this.theta2 = null;
      this.theta3 = null;
      this.theta4 = null;
      this.theta5 = null;
    }
    else {
      if (initObj.hasOwnProperty('theta0')) {
        this.theta0 = initObj.theta0
      }
      else {
        this.theta0 = 0.0;
      }
      if (initObj.hasOwnProperty('theta1')) {
        this.theta1 = initObj.theta1
      }
      else {
        this.theta1 = 0.0;
      }
      if (initObj.hasOwnProperty('theta2')) {
        this.theta2 = initObj.theta2
      }
      else {
        this.theta2 = 0.0;
      }
      if (initObj.hasOwnProperty('theta3')) {
        this.theta3 = initObj.theta3
      }
      else {
        this.theta3 = 0.0;
      }
      if (initObj.hasOwnProperty('theta4')) {
        this.theta4 = initObj.theta4
      }
      else {
        this.theta4 = 0.0;
      }
      if (initObj.hasOwnProperty('theta5')) {
        this.theta5 = initObj.theta5
      }
      else {
        this.theta5 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InverseKinematicResponse
    // Serialize message field [theta0]
    bufferOffset = _serializer.float64(obj.theta0, buffer, bufferOffset);
    // Serialize message field [theta1]
    bufferOffset = _serializer.float64(obj.theta1, buffer, bufferOffset);
    // Serialize message field [theta2]
    bufferOffset = _serializer.float64(obj.theta2, buffer, bufferOffset);
    // Serialize message field [theta3]
    bufferOffset = _serializer.float64(obj.theta3, buffer, bufferOffset);
    // Serialize message field [theta4]
    bufferOffset = _serializer.float64(obj.theta4, buffer, bufferOffset);
    // Serialize message field [theta5]
    bufferOffset = _serializer.float64(obj.theta5, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InverseKinematicResponse
    let len;
    let data = new InverseKinematicResponse(null);
    // Deserialize message field [theta0]
    data.theta0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta1]
    data.theta1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta2]
    data.theta2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta3]
    data.theta3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta4]
    data.theta4 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta5]
    data.theta5 = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur5_pkg/InverseKinematicResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '06829cf9941b7416f2e021ddaa9f0142';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 theta0
    float64 theta1
    float64 theta2
    float64 theta3
    float64 theta4
    float64 theta5
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InverseKinematicResponse(null);
    if (msg.theta0 !== undefined) {
      resolved.theta0 = msg.theta0;
    }
    else {
      resolved.theta0 = 0.0
    }

    if (msg.theta1 !== undefined) {
      resolved.theta1 = msg.theta1;
    }
    else {
      resolved.theta1 = 0.0
    }

    if (msg.theta2 !== undefined) {
      resolved.theta2 = msg.theta2;
    }
    else {
      resolved.theta2 = 0.0
    }

    if (msg.theta3 !== undefined) {
      resolved.theta3 = msg.theta3;
    }
    else {
      resolved.theta3 = 0.0
    }

    if (msg.theta4 !== undefined) {
      resolved.theta4 = msg.theta4;
    }
    else {
      resolved.theta4 = 0.0
    }

    if (msg.theta5 !== undefined) {
      resolved.theta5 = msg.theta5;
    }
    else {
      resolved.theta5 = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: InverseKinematicRequest,
  Response: InverseKinematicResponse,
  md5sum() { return 'c7202f6e227428a04da015ed4eae1b79'; },
  datatype() { return 'ur5_pkg/InverseKinematic'; }
};
