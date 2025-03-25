; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude MACAddress.msg.html

(cl:defclass <MACAddress> (roslisp-msg-protocol:ros-message)
  ((mac_address
    :reader mac_address
    :initarg :mac_address
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass MACAddress (<MACAddress>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MACAddress>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MACAddress)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<MACAddress> is deprecated: use kortex_driver-msg:MACAddress instead.")))

(cl:ensure-generic-function 'mac_address-val :lambda-list '(m))
(cl:defmethod mac_address-val ((m <MACAddress>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:mac_address-val is deprecated.  Use kortex_driver-msg:mac_address instead.")
  (mac_address m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MACAddress>) ostream)
  "Serializes a message object of type '<MACAddress>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mac_address))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'mac_address))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MACAddress>) istream)
  "Deserializes a message object of type '<MACAddress>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mac_address) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mac_address)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MACAddress>)))
  "Returns string type for a message object of type '<MACAddress>"
  "kortex_driver/MACAddress")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MACAddress)))
  "Returns string type for a message object of type 'MACAddress"
  "kortex_driver/MACAddress")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MACAddress>)))
  "Returns md5sum for a message object of type '<MACAddress>"
  "5d1c55e7218bc267e43e7230dde48c16")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MACAddress)))
  "Returns md5sum for a message object of type 'MACAddress"
  "5d1c55e7218bc267e43e7230dde48c16")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MACAddress>)))
  "Returns full string definition for message of type '<MACAddress>"
  (cl:format cl:nil "~%uint8[] mac_address~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MACAddress)))
  "Returns full string definition for message of type 'MACAddress"
  (cl:format cl:nil "~%uint8[] mac_address~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MACAddress>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mac_address) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MACAddress>))
  "Converts a ROS message object to a list"
  (cl:list 'MACAddress
    (cl:cons ':mac_address (mac_address msg))
))
