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

class find_min_arrayRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.array = null;
    }
    else {
      if (initObj.hasOwnProperty('array')) {
        this.array = initObj.array
      }
      else {
        this.array = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type find_min_arrayRequest
    // Serialize message field [array]
    bufferOffset = _arraySerializer.float32(obj.array, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type find_min_arrayRequest
    let len;
    let data = new find_min_arrayRequest(null);
    // Deserialize message field [array]
    data.array = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.array.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ros_exercises/find_min_arrayRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '71f1005c81b671681646a574c6360c24';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] array
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new find_min_arrayRequest(null);
    if (msg.array !== undefined) {
      resolved.array = msg.array;
    }
    else {
      resolved.array = []
    }

    return resolved;
    }
};

class find_min_arrayResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.min = null;
    }
    else {
      if (initObj.hasOwnProperty('min')) {
        this.min = initObj.min
      }
      else {
        this.min = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type find_min_arrayResponse
    // Serialize message field [min]
    bufferOffset = _serializer.float32(obj.min, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type find_min_arrayResponse
    let len;
    let data = new find_min_arrayResponse(null);
    // Deserialize message field [min]
    data.min = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ros_exercises/find_min_arrayResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7bb7e81bbecf53f22e90b5a424f4cb01';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 min
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new find_min_arrayResponse(null);
    if (msg.min !== undefined) {
      resolved.min = msg.min;
    }
    else {
      resolved.min = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: find_min_arrayRequest,
  Response: find_min_arrayResponse,
  md5sum() { return 'c14a13c8d67ff0e22069dfcdf7c3d804'; },
  datatype() { return 'ros_exercises/find_min_array'; }
};
