; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude WifiEnableState.msg.html

(cl:defclass <WifiEnableState> (roslisp-msg-protocol:ros-message)
  ((enabled
    :reader enabled
    :initarg :enabled
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass WifiEnableState (<WifiEnableState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WifiEnableState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WifiEnableState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<WifiEnableState> is deprecated: use kortex_driver-msg:WifiEnableState instead.")))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <WifiEnableState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:enabled-val is deprecated.  Use kortex_driver-msg:enabled instead.")
  (enabled m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WifiEnableState>) ostream)
  "Serializes a message object of type '<WifiEnableState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enabled) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WifiEnableState>) istream)
  "Deserializes a message object of type '<WifiEnableState>"
    (cl:setf (cl:slot-value msg 'enabled) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WifiEnableState>)))
  "Returns string type for a message object of type '<WifiEnableState>"
  "kortex_driver/WifiEnableState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WifiEnableState)))
  "Returns string type for a message object of type 'WifiEnableState"
  "kortex_driver/WifiEnableState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WifiEnableState>)))
  "Returns md5sum for a message object of type '<WifiEnableState>"
  "2815464f55ab63684cc1bc38072d0b9b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WifiEnableState)))
  "Returns md5sum for a message object of type 'WifiEnableState"
  "2815464f55ab63684cc1bc38072d0b9b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WifiEnableState>)))
  "Returns full string definition for message of type '<WifiEnableState>"
  (cl:format cl:nil "~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WifiEnableState)))
  "Returns full string definition for message of type 'WifiEnableState"
  (cl:format cl:nil "~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WifiEnableState>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WifiEnableState>))
  "Converts a ROS message object to a list"
  (cl:list 'WifiEnableState
    (cl:cons ':enabled (enabled msg))
))
