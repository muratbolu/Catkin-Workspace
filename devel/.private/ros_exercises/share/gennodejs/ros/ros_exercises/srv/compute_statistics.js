// Auto-generated. Do not edit!

// (in-package ros_exercises.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class compute_statisticsRequest {
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
        this.input = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type compute_statisticsRequest
    // Serialize message field [input]
    bufferOffset = _arraySerializer.float32(obj.input, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type compute_statisticsRequest
    let len;
    let data = new compute_statisticsRequest(null);
    // Deserialize message field [input]
    data.input = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.input.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ros_exercises/compute_statisticsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '67001d40e8c46d0f13fef73f0cd86f54';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] input
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new compute_statisticsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = msg.input;
    }
    else {
      resolved.input = []
    }

    return resolved;
    }
};

class compute_statisticsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.median = null;
      this.mode = null;
      this.mean = null;
    }
    else {
      if (initObj.hasOwnProperty('median')) {
        this.median = initObj.median
      }
      else {
        this.median = 0.0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0.0;
      }
      if (initObj.hasOwnProperty('mean')) {
        this.mean = initObj.mean
      }
      else {
        this.mean = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type compute_statisticsResponse
    // Serialize message field [median]
    bufferOffset = _serializer.float32(obj.median, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.float32(obj.mode, buffer, bufferOffset);
    // Serialize message field [mean]
    bufferOffset = _serializer.float32(obj.mean, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type compute_statisticsResponse
    let len;
    let data = new compute_statisticsResponse(null);
    // Deserialize message field [median]
    data.median = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mean]
    data.mean = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ros_exercises/compute_statisticsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e77fe3e8f7ae7330fc10b10697279a21';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 median
    float32 mode
    float32 mean
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new compute_statisticsResponse(null);
    if (msg.median !== undefined) {
      resolved.median = msg.median;
    }
    else {
      resolved.median = 0.0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0.0
    }

    if (msg.mean !== undefined) {
      resolved.mean = msg.mean;
    }
    else {
      resolved.mean = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: compute_statisticsRequest,
  Response: compute_statisticsResponse,
  md5sum() { return '89a0f83e0765b462301612617aaed4ac'; },
  datatype() { return 'ros_exercises/compute_statistics'; }
};
