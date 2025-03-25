; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetCalibrationResult-request.msg.html

(cl:defclass <GetCalibrationResult-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:CalibrationElement
    :initform (cl:make-instance 'kortex_driver-msg:CalibrationElement)))
)

(cl:defclass GetCalibrationResult-request (<GetCalibrationResult-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCalibrationResult-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCalibrationResult-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetCalibrationResult-request> is deprecated: use kortex_driver-srv:GetCalibrationResult-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetCalibrationResult-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCalibrationResult-request>) ostream)
  "Serializes a message object of type '<GetCalibrationResult-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCalibrationResult-request>) istream)
  "Deserializes a message object of type '<GetCalibrationResult-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCalibrationResult-request>)))
  "Returns string type for a service object of type '<GetCalibrationResult-request>"
  "kortex_driver/GetCalibrationResultRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCalibrationResult-request)))
  "Returns string type for a service object of type 'GetCalibrationResult-request"
  "kortex_driver/GetCalibrationResultRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCalibrationResult-request>)))
  "Returns md5sum for a message object of type '<GetCalibrationResult-request>"
  "77b1f6ef7f1d5eace34aa81cf1772705")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCalibrationResult-request)))
  "Returns md5sum for a message object of type 'GetCalibrationResult-request"
  "77b1f6ef7f1d5eace34aa81cf1772705")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCalibrationResult-request>)))
  "Returns full string definition for message of type '<GetCalibrationResult-request>"
  (cl:format cl:nil "CalibrationElement input~%~%================================================================================~%MSG: kortex_driver/CalibrationElement~%~%uint32 calibration_item~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCalibrationResult-request)))
  "Returns full string definition for message of type 'GetCalibrationResult-request"
  (cl:format cl:nil "CalibrationElement input~%~%================================================================================~%MSG: kortex_driver/CalibrationElement~%~%uint32 calibration_item~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCalibrationResult-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCalibrationResult-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCalibrationResult-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetCalibrationResult-response.msg.html

(cl:defclass <GetCalibrationResult-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:CalibrationResult
    :initform (cl:make-instance 'kortex_driver-msg:CalibrationResult)))
)

(cl:defclass GetCalibrationResult-response (<GetCalibrationResult-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCalibrationResult-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCalibrationResult-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetCalibrationResult-response> is deprecated: use kortex_driver-srv:GetCalibrationResult-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetCalibrationResult-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCalibrationResult-response>) ostream)
  "Serializes a message object of type '<GetCalibrationResult-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCalibrationResult-response>) istream)
  "Deserializes a message object of type '<GetCalibrationResult-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCalibrationResult-response>)))
  "Returns string type for a service object of type '<GetCalibrationResult-response>"
  "kortex_driver/GetCalibrationResultResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCalibrationResult-response)))
  "Returns string type for a service object of type 'GetCalibrationResult-response"
  "kortex_driver/GetCalibrationResultResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCalibrationResult-response>)))
  "Returns md5sum for a message object of type '<GetCalibrationResult-response>"
  "77b1f6ef7f1d5eace34aa81cf1772705")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCalibrationResult-response)))
  "Returns md5sum for a message object of type 'GetCalibrationResult-response"
  "77b1f6ef7f1d5eace34aa81cf1772705")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCalibrationResult-response>)))
  "Returns full string definition for message of type '<GetCalibrationResult-response>"
  (cl:format cl:nil "CalibrationResult output~%~%================================================================================~%MSG: kortex_driver/CalibrationResult~%~%uint32 calibration_status~%uint32 calibration_details~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCalibrationResult-response)))
  "Returns full string definition for message of type 'GetCalibrationResult-response"
  (cl:format cl:nil "CalibrationResult output~%~%================================================================================~%MSG: kortex_driver/CalibrationResult~%~%uint32 calibration_status~%uint32 calibration_details~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCalibrationResult-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCalibrationResult-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCalibrationResult-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCalibrationResult)))
  'GetCalibrationResult-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCalibrationResult)))
  'GetCalibrationResult-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCalibrationResult)))
  "Returns string type for a service object of type '<GetCalibrationResult>"
  "kortex_driver/GetCalibrationResult")