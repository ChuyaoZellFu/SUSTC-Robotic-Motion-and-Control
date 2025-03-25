; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CalibrationElement.msg.html

(cl:defclass <CalibrationElement> (roslisp-msg-protocol:ros-message)
  ((calibration_item
    :reader calibration_item
    :initarg :calibration_item
    :type cl:integer
    :initform 0))
)

(cl:defclass CalibrationElement (<CalibrationElement>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalibrationElement>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalibrationElement)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CalibrationElement> is deprecated: use kortex_driver-msg:CalibrationElement instead.")))

(cl:ensure-generic-function 'calibration_item-val :lambda-list '(m))
(cl:defmethod calibration_item-val ((m <CalibrationElement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:calibration_item-val is deprecated.  Use kortex_driver-msg:calibration_item instead.")
  (calibration_item m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalibrationElement>) ostream)
  "Serializes a message object of type '<CalibrationElement>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'calibration_item)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'calibration_item)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'calibration_item)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'calibration_item)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalibrationElement>) istream)
  "Deserializes a message object of type '<CalibrationElement>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'calibration_item)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'calibration_item)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'calibration_item)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'calibration_item)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalibrationElement>)))
  "Returns string type for a message object of type '<CalibrationElement>"
  "kortex_driver/CalibrationElement")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalibrationElement)))
  "Returns string type for a message object of type 'CalibrationElement"
  "kortex_driver/CalibrationElement")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalibrationElement>)))
  "Returns md5sum for a message object of type '<CalibrationElement>"
  "48b2b6bb8d2654cfbe2379c075bcb6a4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalibrationElement)))
  "Returns md5sum for a message object of type 'CalibrationElement"
  "48b2b6bb8d2654cfbe2379c075bcb6a4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalibrationElement>)))
  "Returns full string definition for message of type '<CalibrationElement>"
  (cl:format cl:nil "~%uint32 calibration_item~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalibrationElement)))
  "Returns full string definition for message of type 'CalibrationElement"
  (cl:format cl:nil "~%uint32 calibration_item~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalibrationElement>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalibrationElement>))
  "Converts a ROS message object to a list"
  (cl:list 'CalibrationElement
    (cl:cons ':calibration_item (calibration_item msg))
))
