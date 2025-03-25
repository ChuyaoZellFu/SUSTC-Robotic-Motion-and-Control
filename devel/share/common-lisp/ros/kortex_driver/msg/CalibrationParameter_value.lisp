; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CalibrationParameter_value.msg.html

(cl:defclass <CalibrationParameter_value> (roslisp-msg-protocol:ros-message)
  ((signedIntValue
    :reader signedIntValue
    :initarg :signedIntValue
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (unsignedIntValue
    :reader unsignedIntValue
    :initarg :unsignedIntValue
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (floatValue
    :reader floatValue
    :initarg :floatValue
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass CalibrationParameter_value (<CalibrationParameter_value>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalibrationParameter_value>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalibrationParameter_value)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CalibrationParameter_value> is deprecated: use kortex_driver-msg:CalibrationParameter_value instead.")))

(cl:ensure-generic-function 'signedIntValue-val :lambda-list '(m))
(cl:defmethod signedIntValue-val ((m <CalibrationParameter_value>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:signedIntValue-val is deprecated.  Use kortex_driver-msg:signedIntValue instead.")
  (signedIntValue m))

(cl:ensure-generic-function 'unsignedIntValue-val :lambda-list '(m))
(cl:defmethod unsignedIntValue-val ((m <CalibrationParameter_value>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:unsignedIntValue-val is deprecated.  Use kortex_driver-msg:unsignedIntValue instead.")
  (unsignedIntValue m))

(cl:ensure-generic-function 'floatValue-val :lambda-list '(m))
(cl:defmethod floatValue-val ((m <CalibrationParameter_value>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:floatValue-val is deprecated.  Use kortex_driver-msg:floatValue instead.")
  (floatValue m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalibrationParameter_value>) ostream)
  "Serializes a message object of type '<CalibrationParameter_value>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'signedIntValue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'signedIntValue))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'unsignedIntValue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'unsignedIntValue))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'floatValue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'floatValue))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalibrationParameter_value>) istream)
  "Deserializes a message object of type '<CalibrationParameter_value>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'signedIntValue) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'signedIntValue)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'unsignedIntValue) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'unsignedIntValue)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'floatValue) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'floatValue)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalibrationParameter_value>)))
  "Returns string type for a message object of type '<CalibrationParameter_value>"
  "kortex_driver/CalibrationParameter_value")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalibrationParameter_value)))
  "Returns string type for a message object of type 'CalibrationParameter_value"
  "kortex_driver/CalibrationParameter_value")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalibrationParameter_value>)))
  "Returns md5sum for a message object of type '<CalibrationParameter_value>"
  "2b45c1a772c79ef326799a208a383734")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalibrationParameter_value)))
  "Returns md5sum for a message object of type 'CalibrationParameter_value"
  "2b45c1a772c79ef326799a208a383734")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalibrationParameter_value>)))
  "Returns full string definition for message of type '<CalibrationParameter_value>"
  (cl:format cl:nil "~%uint32[] signedIntValue~%uint32[] unsignedIntValue~%uint32[] floatValue~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalibrationParameter_value)))
  "Returns full string definition for message of type 'CalibrationParameter_value"
  (cl:format cl:nil "~%uint32[] signedIntValue~%uint32[] unsignedIntValue~%uint32[] floatValue~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalibrationParameter_value>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'signedIntValue) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'unsignedIntValue) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'floatValue) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalibrationParameter_value>))
  "Converts a ROS message object to a list"
  (cl:list 'CalibrationParameter_value
    (cl:cons ':signedIntValue (signedIntValue msg))
    (cl:cons ':unsignedIntValue (unsignedIntValue msg))
    (cl:cons ':floatValue (floatValue msg))
))
