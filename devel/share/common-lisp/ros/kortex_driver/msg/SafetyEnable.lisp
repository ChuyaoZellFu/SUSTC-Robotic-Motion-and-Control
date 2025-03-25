; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SafetyEnable.msg.html

(cl:defclass <SafetyEnable> (roslisp-msg-protocol:ros-message)
  ((handle
    :reader handle
    :initarg :handle
    :type kortex_driver-msg:SafetyHandle
    :initform (cl:make-instance 'kortex_driver-msg:SafetyHandle))
   (enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SafetyEnable (<SafetyEnable>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SafetyEnable>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SafetyEnable)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SafetyEnable> is deprecated: use kortex_driver-msg:SafetyEnable instead.")))

(cl:ensure-generic-function 'handle-val :lambda-list '(m))
(cl:defmethod handle-val ((m <SafetyEnable>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:handle-val is deprecated.  Use kortex_driver-msg:handle instead.")
  (handle m))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <SafetyEnable>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:enable-val is deprecated.  Use kortex_driver-msg:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SafetyEnable>) ostream)
  "Serializes a message object of type '<SafetyEnable>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handle) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SafetyEnable>) istream)
  "Deserializes a message object of type '<SafetyEnable>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handle) istream)
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SafetyEnable>)))
  "Returns string type for a message object of type '<SafetyEnable>"
  "kortex_driver/SafetyEnable")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SafetyEnable)))
  "Returns string type for a message object of type 'SafetyEnable"
  "kortex_driver/SafetyEnable")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SafetyEnable>)))
  "Returns md5sum for a message object of type '<SafetyEnable>"
  "5f3f3341f0f54081acc9ca67a9de861a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SafetyEnable)))
  "Returns md5sum for a message object of type 'SafetyEnable"
  "5f3f3341f0f54081acc9ca67a9de861a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SafetyEnable>)))
  "Returns full string definition for message of type '<SafetyEnable>"
  (cl:format cl:nil "~%SafetyHandle handle~%bool enable~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SafetyEnable)))
  "Returns full string definition for message of type 'SafetyEnable"
  (cl:format cl:nil "~%SafetyHandle handle~%bool enable~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SafetyEnable>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handle))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SafetyEnable>))
  "Converts a ROS message object to a list"
  (cl:list 'SafetyEnable
    (cl:cons ':handle (handle msg))
    (cl:cons ':enable (enable msg))
))
