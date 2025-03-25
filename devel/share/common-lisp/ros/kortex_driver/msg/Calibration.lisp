; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Calibration.msg.html

(cl:defclass <Calibration> (roslisp-msg-protocol:ros-message)
  ((calibration_item
    :reader calibration_item
    :initarg :calibration_item
    :type cl:integer
    :initform 0)
   (calibration_parameter
    :reader calibration_parameter
    :initarg :calibration_parameter
    :type (cl:vector kortex_driver-msg:CalibrationParameter)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:CalibrationParameter :initial-element (cl:make-instance 'kortex_driver-msg:CalibrationParameter))))
)

(cl:defclass Calibration (<Calibration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Calibration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Calibration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Calibration> is deprecated: use kortex_driver-msg:Calibration instead.")))

(cl:ensure-generic-function 'calibration_item-val :lambda-list '(m))
(cl:defmethod calibration_item-val ((m <Calibration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:calibration_item-val is deprecated.  Use kortex_driver-msg:calibration_item instead.")
  (calibration_item m))

(cl:ensure-generic-function 'calibration_parameter-val :lambda-list '(m))
(cl:defmethod calibration_parameter-val ((m <Calibration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:calibration_parameter-val is deprecated.  Use kortex_driver-msg:calibration_parameter instead.")
  (calibration_parameter m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Calibration>) ostream)
  "Serializes a message object of type '<Calibration>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'calibration_item)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'calibration_item)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'calibration_item)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'calibration_item)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'calibration_parameter))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'calibration_parameter))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Calibration>) istream)
  "Deserializes a message object of type '<Calibration>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'calibration_item)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'calibration_item)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'calibration_item)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'calibration_item)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'calibration_parameter) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'calibration_parameter)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:CalibrationParameter))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Calibration>)))
  "Returns string type for a message object of type '<Calibration>"
  "kortex_driver/Calibration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Calibration)))
  "Returns string type for a message object of type 'Calibration"
  "kortex_driver/Calibration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Calibration>)))
  "Returns md5sum for a message object of type '<Calibration>"
  "4e1f62c964640a6b04e3e0de4bfef9f2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Calibration)))
  "Returns md5sum for a message object of type 'Calibration"
  "4e1f62c964640a6b04e3e0de4bfef9f2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Calibration>)))
  "Returns full string definition for message of type '<Calibration>"
  (cl:format cl:nil "~%uint32 calibration_item~%CalibrationParameter[] calibration_parameter~%================================================================================~%MSG: kortex_driver/CalibrationParameter~%~%uint32 calibration_parameter_identifier~%CalibrationParameter_value oneof_value~%================================================================================~%MSG: kortex_driver/CalibrationParameter_value~%~%uint32[] signedIntValue~%uint32[] unsignedIntValue~%uint32[] floatValue~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Calibration)))
  "Returns full string definition for message of type 'Calibration"
  (cl:format cl:nil "~%uint32 calibration_item~%CalibrationParameter[] calibration_parameter~%================================================================================~%MSG: kortex_driver/CalibrationParameter~%~%uint32 calibration_parameter_identifier~%CalibrationParameter_value oneof_value~%================================================================================~%MSG: kortex_driver/CalibrationParameter_value~%~%uint32[] signedIntValue~%uint32[] unsignedIntValue~%uint32[] floatValue~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Calibration>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'calibration_parameter) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Calibration>))
  "Converts a ROS message object to a list"
  (cl:list 'Calibration
    (cl:cons ':calibration_item (calibration_item msg))
    (cl:cons ':calibration_parameter (calibration_parameter msg))
))
