; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CalibrationParameter.msg.html

(cl:defclass <CalibrationParameter> (roslisp-msg-protocol:ros-message)
  ((calibration_parameter_identifier
    :reader calibration_parameter_identifier
    :initarg :calibration_parameter_identifier
    :type cl:integer
    :initform 0)
   (oneof_value
    :reader oneof_value
    :initarg :oneof_value
    :type kortex_driver-msg:CalibrationParameter_value
    :initform (cl:make-instance 'kortex_driver-msg:CalibrationParameter_value)))
)

(cl:defclass CalibrationParameter (<CalibrationParameter>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalibrationParameter>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalibrationParameter)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CalibrationParameter> is deprecated: use kortex_driver-msg:CalibrationParameter instead.")))

(cl:ensure-generic-function 'calibration_parameter_identifier-val :lambda-list '(m))
(cl:defmethod calibration_parameter_identifier-val ((m <CalibrationParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:calibration_parameter_identifier-val is deprecated.  Use kortex_driver-msg:calibration_parameter_identifier instead.")
  (calibration_parameter_identifier m))

(cl:ensure-generic-function 'oneof_value-val :lambda-list '(m))
(cl:defmethod oneof_value-val ((m <CalibrationParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:oneof_value-val is deprecated.  Use kortex_driver-msg:oneof_value instead.")
  (oneof_value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalibrationParameter>) ostream)
  "Serializes a message object of type '<CalibrationParameter>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'calibration_parameter_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'calibration_parameter_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'calibration_parameter_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'calibration_parameter_identifier)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'oneof_value) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalibrationParameter>) istream)
  "Deserializes a message object of type '<CalibrationParameter>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'calibration_parameter_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'calibration_parameter_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'calibration_parameter_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'calibration_parameter_identifier)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'oneof_value) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalibrationParameter>)))
  "Returns string type for a message object of type '<CalibrationParameter>"
  "kortex_driver/CalibrationParameter")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalibrationParameter)))
  "Returns string type for a message object of type 'CalibrationParameter"
  "kortex_driver/CalibrationParameter")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalibrationParameter>)))
  "Returns md5sum for a message object of type '<CalibrationParameter>"
  "c9d31a3bff291563e3d32bb702a8bed2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalibrationParameter)))
  "Returns md5sum for a message object of type 'CalibrationParameter"
  "c9d31a3bff291563e3d32bb702a8bed2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalibrationParameter>)))
  "Returns full string definition for message of type '<CalibrationParameter>"
  (cl:format cl:nil "~%uint32 calibration_parameter_identifier~%CalibrationParameter_value oneof_value~%================================================================================~%MSG: kortex_driver/CalibrationParameter_value~%~%uint32[] signedIntValue~%uint32[] unsignedIntValue~%uint32[] floatValue~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalibrationParameter)))
  "Returns full string definition for message of type 'CalibrationParameter"
  (cl:format cl:nil "~%uint32 calibration_parameter_identifier~%CalibrationParameter_value oneof_value~%================================================================================~%MSG: kortex_driver/CalibrationParameter_value~%~%uint32[] signedIntValue~%uint32[] unsignedIntValue~%uint32[] floatValue~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalibrationParameter>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'oneof_value))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalibrationParameter>))
  "Converts a ROS message object to a list"
  (cl:list 'CalibrationParameter
    (cl:cons ':calibration_parameter_identifier (calibration_parameter_identifier msg))
    (cl:cons ':oneof_value (oneof_value msg))
))
