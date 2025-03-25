; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ExecuteCalibration-request.msg.html

(cl:defclass <ExecuteCalibration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Calibration
    :initform (cl:make-instance 'kortex_driver-msg:Calibration)))
)

(cl:defclass ExecuteCalibration-request (<ExecuteCalibration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteCalibration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteCalibration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ExecuteCalibration-request> is deprecated: use kortex_driver-srv:ExecuteCalibration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ExecuteCalibration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteCalibration-request>) ostream)
  "Serializes a message object of type '<ExecuteCalibration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteCalibration-request>) istream)
  "Deserializes a message object of type '<ExecuteCalibration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteCalibration-request>)))
  "Returns string type for a service object of type '<ExecuteCalibration-request>"
  "kortex_driver/ExecuteCalibrationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteCalibration-request)))
  "Returns string type for a service object of type 'ExecuteCalibration-request"
  "kortex_driver/ExecuteCalibrationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteCalibration-request>)))
  "Returns md5sum for a message object of type '<ExecuteCalibration-request>"
  "8aabe5c823e6d1cd3d6eddd247265dae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteCalibration-request)))
  "Returns md5sum for a message object of type 'ExecuteCalibration-request"
  "8aabe5c823e6d1cd3d6eddd247265dae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteCalibration-request>)))
  "Returns full string definition for message of type '<ExecuteCalibration-request>"
  (cl:format cl:nil "Calibration input~%~%================================================================================~%MSG: kortex_driver/Calibration~%~%uint32 calibration_item~%CalibrationParameter[] calibration_parameter~%================================================================================~%MSG: kortex_driver/CalibrationParameter~%~%uint32 calibration_parameter_identifier~%CalibrationParameter_value oneof_value~%================================================================================~%MSG: kortex_driver/CalibrationParameter_value~%~%uint32[] signedIntValue~%uint32[] unsignedIntValue~%uint32[] floatValue~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteCalibration-request)))
  "Returns full string definition for message of type 'ExecuteCalibration-request"
  (cl:format cl:nil "Calibration input~%~%================================================================================~%MSG: kortex_driver/Calibration~%~%uint32 calibration_item~%CalibrationParameter[] calibration_parameter~%================================================================================~%MSG: kortex_driver/CalibrationParameter~%~%uint32 calibration_parameter_identifier~%CalibrationParameter_value oneof_value~%================================================================================~%MSG: kortex_driver/CalibrationParameter_value~%~%uint32[] signedIntValue~%uint32[] unsignedIntValue~%uint32[] floatValue~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteCalibration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteCalibration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteCalibration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ExecuteCalibration-response.msg.html

(cl:defclass <ExecuteCalibration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ExecuteCalibration-response (<ExecuteCalibration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteCalibration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteCalibration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ExecuteCalibration-response> is deprecated: use kortex_driver-srv:ExecuteCalibration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ExecuteCalibration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteCalibration-response>) ostream)
  "Serializes a message object of type '<ExecuteCalibration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteCalibration-response>) istream)
  "Deserializes a message object of type '<ExecuteCalibration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteCalibration-response>)))
  "Returns string type for a service object of type '<ExecuteCalibration-response>"
  "kortex_driver/ExecuteCalibrationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteCalibration-response)))
  "Returns string type for a service object of type 'ExecuteCalibration-response"
  "kortex_driver/ExecuteCalibrationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteCalibration-response>)))
  "Returns md5sum for a message object of type '<ExecuteCalibration-response>"
  "8aabe5c823e6d1cd3d6eddd247265dae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteCalibration-response)))
  "Returns md5sum for a message object of type 'ExecuteCalibration-response"
  "8aabe5c823e6d1cd3d6eddd247265dae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteCalibration-response>)))
  "Returns full string definition for message of type '<ExecuteCalibration-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteCalibration-response)))
  "Returns full string definition for message of type 'ExecuteCalibration-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteCalibration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteCalibration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteCalibration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ExecuteCalibration)))
  'ExecuteCalibration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ExecuteCalibration)))
  'ExecuteCalibration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteCalibration)))
  "Returns string type for a service object of type '<ExecuteCalibration>"
  "kortex_driver/ExecuteCalibration")