; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude DeviceHandles.msg.html

(cl:defclass <DeviceHandles> (roslisp-msg-protocol:ros-message)
  ((device_handle
    :reader device_handle
    :initarg :device_handle
    :type (cl:vector kortex_driver-msg:DeviceHandle)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:DeviceHandle :initial-element (cl:make-instance 'kortex_driver-msg:DeviceHandle))))
)

(cl:defclass DeviceHandles (<DeviceHandles>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeviceHandles>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeviceHandles)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<DeviceHandles> is deprecated: use kortex_driver-msg:DeviceHandles instead.")))

(cl:ensure-generic-function 'device_handle-val :lambda-list '(m))
(cl:defmethod device_handle-val ((m <DeviceHandles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device_handle-val is deprecated.  Use kortex_driver-msg:device_handle instead.")
  (device_handle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeviceHandles>) ostream)
  "Serializes a message object of type '<DeviceHandles>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'device_handle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'device_handle))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeviceHandles>) istream)
  "Deserializes a message object of type '<DeviceHandles>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'device_handle) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'device_handle)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:DeviceHandle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeviceHandles>)))
  "Returns string type for a message object of type '<DeviceHandles>"
  "kortex_driver/DeviceHandles")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceHandles)))
  "Returns string type for a message object of type 'DeviceHandles"
  "kortex_driver/DeviceHandles")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeviceHandles>)))
  "Returns md5sum for a message object of type '<DeviceHandles>"
  "ad854635e0d722ce08228e35a86c0173")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeviceHandles)))
  "Returns md5sum for a message object of type 'DeviceHandles"
  "ad854635e0d722ce08228e35a86c0173")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeviceHandles>)))
  "Returns full string definition for message of type '<DeviceHandles>"
  (cl:format cl:nil "~%DeviceHandle[] device_handle~%================================================================================~%MSG: kortex_driver/DeviceHandle~%~%uint32 device_type~%uint32 device_identifier~%uint32 order~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeviceHandles)))
  "Returns full string definition for message of type 'DeviceHandles"
  (cl:format cl:nil "~%DeviceHandle[] device_handle~%================================================================================~%MSG: kortex_driver/DeviceHandle~%~%uint32 device_type~%uint32 device_identifier~%uint32 order~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeviceHandles>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'device_handle) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeviceHandles>))
  "Converts a ROS message object to a list"
  (cl:list 'DeviceHandles
    (cl:cons ':device_handle (device_handle msg))
))
