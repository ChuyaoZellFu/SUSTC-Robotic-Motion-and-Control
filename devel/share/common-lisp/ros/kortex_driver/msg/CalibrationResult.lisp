; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CalibrationResult.msg.html

(cl:defclass <CalibrationResult> (roslisp-msg-protocol:ros-message)
  ((calibration_status
    :reader calibration_status
    :initarg :calibration_status
    :type cl:integer
    :initform 0)
   (calibration_details
    :reader calibration_details
    :initarg :calibration_details
    :type cl:integer
    :initform 0))
)

(cl:defclass CalibrationResult (<CalibrationResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalibrationResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalibrationResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CalibrationResult> is deprecated: use kortex_driver-msg:CalibrationResult instead.")))

(cl:ensure-generic-function 'calibration_status-val :lambda-list '(m))
(cl:defmethod calibration_status-val ((m <CalibrationResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:calibration_status-val is deprecated.  Use kortex_driver-msg:calibration_status instead.")
  (calibration_status m))

(cl:ensure-generic-function 'calibration_details-val :lambda-list '(m))
(cl:defmethod calibration_details-val ((m <CalibrationResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:calibration_details-val is deprecated.  Use kortex_driver-msg:calibration_details instead.")
  (calibration_details m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalibrationResult>) ostream)
  "Serializes a message object of type '<CalibrationResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'calibration_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'calibration_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'calibration_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'calibration_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'calibration_details)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'calibration_details)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'calibration_details)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'calibration_details)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalibrationResult>) istream)
  "Deserializes a message object of type '<CalibrationResult>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'calibration_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'calibration_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'calibration_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'calibration_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'calibration_details)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'calibration_details)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'calibration_details)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'calibration_details)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalibrationResult>)))
  "Returns string type for a message object of type '<CalibrationResult>"
  "kortex_driver/CalibrationResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalibrationResult)))
  "Returns string type for a message object of type 'CalibrationResult"
  "kortex_driver/CalibrationResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalibrationResult>)))
  "Returns md5sum for a message object of type '<CalibrationResult>"
  "3d6092aff91a3268dc7e1b71a52b5cd9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalibrationResult)))
  "Returns md5sum for a message object of type 'CalibrationResult"
  "3d6092aff91a3268dc7e1b71a52b5cd9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalibrationResult>)))
  "Returns full string definition for message of type '<CalibrationResult>"
  (cl:format cl:nil "~%uint32 calibration_status~%uint32 calibration_details~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalibrationResult)))
  "Returns full string definition for message of type 'CalibrationResult"
  (cl:format cl:nil "~%uint32 calibration_status~%uint32 calibration_details~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalibrationResult>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalibrationResult>))
  "Converts a ROS message object to a list"
  (cl:list 'CalibrationResult
    (cl:cons ':calibration_status (calibration_status msg))
    (cl:cons ':calibration_details (calibration_details msg))
))
