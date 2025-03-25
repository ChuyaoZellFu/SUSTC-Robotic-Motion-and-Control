; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BluetoothEnableState.msg.html

(cl:defclass <BluetoothEnableState> (roslisp-msg-protocol:ros-message)
  ((enabled
    :reader enabled
    :initarg :enabled
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass BluetoothEnableState (<BluetoothEnableState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BluetoothEnableState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BluetoothEnableState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BluetoothEnableState> is deprecated: use kortex_driver-msg:BluetoothEnableState instead.")))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <BluetoothEnableState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:enabled-val is deprecated.  Use kortex_driver-msg:enabled instead.")
  (enabled m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BluetoothEnableState>) ostream)
  "Serializes a message object of type '<BluetoothEnableState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enabled) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BluetoothEnableState>) istream)
  "Deserializes a message object of type '<BluetoothEnableState>"
    (cl:setf (cl:slot-value msg 'enabled) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BluetoothEnableState>)))
  "Returns string type for a message object of type '<BluetoothEnableState>"
  "kortex_driver/BluetoothEnableState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BluetoothEnableState)))
  "Returns string type for a message object of type 'BluetoothEnableState"
  "kortex_driver/BluetoothEnableState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BluetoothEnableState>)))
  "Returns md5sum for a message object of type '<BluetoothEnableState>"
  "2815464f55ab63684cc1bc38072d0b9b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BluetoothEnableState)))
  "Returns md5sum for a message object of type 'BluetoothEnableState"
  "2815464f55ab63684cc1bc38072d0b9b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BluetoothEnableState>)))
  "Returns full string definition for message of type '<BluetoothEnableState>"
  (cl:format cl:nil "~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BluetoothEnableState)))
  "Returns full string definition for message of type 'BluetoothEnableState"
  (cl:format cl:nil "~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BluetoothEnableState>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BluetoothEnableState>))
  "Converts a ROS message object to a list"
  (cl:list 'BluetoothEnableState
    (cl:cons ':enabled (enabled msg))
))
