; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude I2CData.msg.html

(cl:defclass <I2CData> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (size
    :reader size
    :initarg :size
    :type cl:integer
    :initform 0))
)

(cl:defclass I2CData (<I2CData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<I2CData> is deprecated: use kortex_driver-msg:I2CData instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <I2CData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:data-val is deprecated.  Use kortex_driver-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <I2CData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:size-val is deprecated.  Use kortex_driver-msg:size instead.")
  (size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CData>) ostream)
  "Serializes a message object of type '<I2CData>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'size)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CData>) istream)
  "Deserializes a message object of type '<I2CData>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'size)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CData>)))
  "Returns string type for a message object of type '<I2CData>"
  "kortex_driver/I2CData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CData)))
  "Returns string type for a message object of type 'I2CData"
  "kortex_driver/I2CData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CData>)))
  "Returns md5sum for a message object of type '<I2CData>"
  "3abc3c9d46f09b6c612dbaa8aeebf087")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CData)))
  "Returns md5sum for a message object of type 'I2CData"
  "3abc3c9d46f09b6c612dbaa8aeebf087")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CData>)))
  "Returns full string definition for message of type '<I2CData>"
  (cl:format cl:nil "~%uint8[] data~%uint32 size~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CData)))
  "Returns full string definition for message of type 'I2CData"
  (cl:format cl:nil "~%uint8[] data~%uint32 size~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CData>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CData>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CData
    (cl:cons ':data (data msg))
    (cl:cons ':size (size msg))
))
