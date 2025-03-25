; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude StopCalibration-request.msg.html

(cl:defclass <StopCalibration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Calibration
    :initform (cl:make-instance 'kortex_driver-msg:Calibration)))
)

(cl:defclass StopCalibration-request (<StopCalibration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopCalibration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopCalibration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<StopCalibration-request> is deprecated: use kortex_driver-srv:StopCalibration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <StopCalibration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopCalibration-request>) ostream)
  "Serializes a message object of type '<StopCalibration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopCalibration-request>) istream)
  "Deserializes a message object of type '<StopCalibration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopCalibration-request>)))
  "Returns string type for a service object of type '<StopCalibration-request>"
  "kortex_driver/StopCalibrationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopCalibration-request)))
  "Returns string type for a service object of type 'StopCalibration-request"
  "kortex_driver/StopCalibrationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopCalibration-request>)))
  "Returns md5sum for a message object of type '<StopCalibration-request>"
  "d0dfa8caa770ffbbffd12276bb3d7f2c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopCalibration-request)))
  "Returns md5sum for a message object of type 'StopCalibration-request"
  "d0dfa8caa770ffbbffd12276bb3d7f2c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopCalibration-request>)))
  "Returns full string definition for message of type '<StopCalibration-request>"
  (cl:format cl:nil "Calibration input~%~%================================================================================~%MSG: kortex_driver/Calibration~%~%uint32 calibration_item~%CalibrationParameter[] calibration_parameter~%================================================================================~%MSG: kortex_driver/CalibrationParameter~%~%uint32 calibration_parameter_identifier~%CalibrationParameter_value oneof_value~%================================================================================~%MSG: kortex_driver/CalibrationParameter_value~%~%uint32[] signedIntValue~%uint32[] unsignedIntValue~%uint32[] floatValue~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopCalibration-request)))
  "Returns full string definition for message of type 'StopCalibration-request"
  (cl:format cl:nil "Calibration input~%~%================================================================================~%MSG: kortex_driver/Calibration~%~%uint32 calibration_item~%CalibrationParameter[] calibration_parameter~%================================================================================~%MSG: kortex_driver/CalibrationParameter~%~%uint32 calibration_parameter_identifier~%CalibrationParameter_value oneof_value~%================================================================================~%MSG: kortex_driver/CalibrationParameter_value~%~%uint32[] signedIntValue~%uint32[] unsignedIntValue~%uint32[] floatValue~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopCalibration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopCalibration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StopCalibration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude StopCalibration-response.msg.html

(cl:defclass <StopCalibration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:CalibrationResult
    :initform (cl:make-instance 'kortex_driver-msg:CalibrationResult)))
)

(cl:defclass StopCalibration-response (<StopCalibration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopCalibration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopCalibration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<StopCalibration-response> is deprecated: use kortex_driver-srv:StopCalibration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <StopCalibration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopCalibration-response>) ostream)
  "Serializes a message object of type '<StopCalibration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopCalibration-response>) istream)
  "Deserializes a message object of type '<StopCalibration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopCalibration-response>)))
  "Returns string type for a service object of type '<StopCalibration-response>"
  "kortex_driver/StopCalibrationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopCalibration-response)))
  "Returns string type for a service object of type 'StopCalibration-response"
  "kortex_driver/StopCalibrationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopCalibration-response>)))
  "Returns md5sum for a message object of type '<StopCalibration-response>"
  "d0dfa8caa770ffbbffd12276bb3d7f2c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopCalibration-response)))
  "Returns md5sum for a message object of type 'StopCalibration-response"
  "d0dfa8caa770ffbbffd12276bb3d7f2c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopCalibration-response>)))
  "Returns full string definition for message of type '<StopCalibration-response>"
  (cl:format cl:nil "CalibrationResult output~%~%================================================================================~%MSG: kortex_driver/CalibrationResult~%~%uint32 calibration_status~%uint32 calibration_details~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopCalibration-response)))
  "Returns full string definition for message of type 'StopCalibration-response"
  (cl:format cl:nil "CalibrationResult output~%~%================================================================================~%MSG: kortex_driver/CalibrationResult~%~%uint32 calibration_status~%uint32 calibration_details~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopCalibration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopCalibration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StopCalibration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StopCalibration)))
  'StopCalibration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StopCalibration)))
  'StopCalibration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopCalibration)))
  "Returns string type for a service object of type '<StopCalibration>"
  "kortex_driver/StopCalibration")