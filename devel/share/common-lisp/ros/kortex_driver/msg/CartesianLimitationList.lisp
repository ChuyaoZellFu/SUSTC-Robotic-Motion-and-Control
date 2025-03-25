; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CartesianLimitationList.msg.html

(cl:defclass <CartesianLimitationList> (roslisp-msg-protocol:ros-message)
  ((limitations
    :reader limitations
    :initarg :limitations
    :type (cl:vector kortex_driver-msg:CartesianLimitation)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:CartesianLimitation :initial-element (cl:make-instance 'kortex_driver-msg:CartesianLimitation))))
)

(cl:defclass CartesianLimitationList (<CartesianLimitationList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartesianLimitationList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartesianLimitationList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CartesianLimitationList> is deprecated: use kortex_driver-msg:CartesianLimitationList instead.")))

(cl:ensure-generic-function 'limitations-val :lambda-list '(m))
(cl:defmethod limitations-val ((m <CartesianLimitationList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:limitations-val is deprecated.  Use kortex_driver-msg:limitations instead.")
  (limitations m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartesianLimitationList>) ostream)
  "Serializes a message object of type '<CartesianLimitationList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'limitations))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'limitations))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartesianLimitationList>) istream)
  "Deserializes a message object of type '<CartesianLimitationList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'limitations) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'limitations)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:CartesianLimitation))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartesianLimitationList>)))
  "Returns string type for a message object of type '<CartesianLimitationList>"
  "kortex_driver/CartesianLimitationList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartesianLimitationList)))
  "Returns string type for a message object of type 'CartesianLimitationList"
  "kortex_driver/CartesianLimitationList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartesianLimitationList>)))
  "Returns md5sum for a message object of type '<CartesianLimitationList>"
  "c96ad5571d257cbcdf15809b29c600bb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartesianLimitationList)))
  "Returns md5sum for a message object of type 'CartesianLimitationList"
  "c96ad5571d257cbcdf15809b29c600bb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartesianLimitationList>)))
  "Returns full string definition for message of type '<CartesianLimitationList>"
  (cl:format cl:nil "~%CartesianLimitation[] limitations~%================================================================================~%MSG: kortex_driver/CartesianLimitation~%~%uint32 type~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartesianLimitationList)))
  "Returns full string definition for message of type 'CartesianLimitationList"
  (cl:format cl:nil "~%CartesianLimitation[] limitations~%================================================================================~%MSG: kortex_driver/CartesianLimitation~%~%uint32 type~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartesianLimitationList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'limitations) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartesianLimitationList>))
  "Converts a ROS message object to a list"
  (cl:list 'CartesianLimitationList
    (cl:cons ':limitations (limitations msg))
))
