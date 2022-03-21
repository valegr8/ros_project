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

class JacobianKinematicRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input = null;
    }
    else {
      if (initObj.hasOwnProperty('input')) {
        this.input = initObj.input
      }
      else {
        this.input = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JacobianKinematicRequest
    // Serialize message field [input]
    bufferOffset = _serializer.float64(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JacobianKinematicRequest
    let len;
    let data = new JacobianKinematicRequest(null);
    // Deserialize message field [input]
    data.input = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur5_pkg/JacobianKinematicRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '235c8ad2b88a9725a5a2ad2a9541a007';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 input
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JacobianKinematicRequest(null);
    if (msg.input !== undefined) {
      resolved.input = msg.input;
    }
    else {
      resolved.input = 0.0
    }

    return resolved;
    }
};

class JacobianKinematicResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.output = null;
    }
    else {
      if (initObj.hasOwnProperty('output')) {
        this.output = initObj.output
      }
      else {
        this.output = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JacobianKinematicResponse
    // Serialize message field [output]
    bufferOffset = _serializer.float64(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JacobianKinematicResponse
    let len;
    let data = new JacobianKinematicResponse(null);
    // Deserialize message field [output]
    data.output = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur5_pkg/JacobianKinematicResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5dd87a43ba76105996c6c8cafb738498';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 output
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JacobianKinematicResponse(null);
    if (msg.output !== undefined) {
      resolved.output = msg.output;
    }
    else {
      resolved.output = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: JacobianKinematicRequest,
  Response: JacobianKinematicResponse,
  md5sum() { return '6c59364dede48a4429627e3e0efa7049'; },
  datatype() { return 'ur5_pkg/JacobianKinematic'; }
};
