; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Servoing.msg.html

(cl:defclass <Servoing> (roslisp-msg-protocol:ros-message)
  ((enabled
    :reader enabled
    :initarg :enabled
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Servoing (<Servoing>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Servoing>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Servoing)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Servoing> is deprecated: use kortex_driver-msg:Servoing instead.")))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <Servoing>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:enabled-val is deprecated.  Use kortex_driver-msg:enabled instead.")
  (enabled m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Servoing>) ostream)
  "Serializes a message object of type '<Servoing>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enabled) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Servoing>) istream)
  "Deserializes a message object of type '<Servoing>"
    (cl:setf (cl:slot-value msg 'enabled) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Servoing>)))
  "Returns string type for a message object of type '<Servoing>"
  "kortex_driver/Servoing")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Servoing)))
  "Returns string type for a message object of type 'Servoing"
  "kortex_driver/Servoing")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Servoing>)))
  "Returns md5sum for a message object of type '<Servoing>"
  "2815464f55ab63684cc1bc38072d0b9b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Servoing)))
  "Returns md5sum for a message object of type 'Servoing"
  "2815464f55ab63684cc1bc38072d0b9b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Servoing>)))
  "Returns full string definition for message of type '<Servoing>"
  (cl:format cl:nil "~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Servoing)))
  "Returns full string definition for message of type 'Servoing"
  (cl:format cl:nil "~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Servoing>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Servoing>))
  "Converts a ROS message object to a list"
  (cl:list 'Servoing
    (cl:cons ':enabled (enabled msg))
))
