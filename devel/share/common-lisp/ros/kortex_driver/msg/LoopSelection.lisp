; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude LoopSelection.msg.html

(cl:defclass <LoopSelection> (roslisp-msg-protocol:ros-message)
  ((loop_selection
    :reader loop_selection
    :initarg :loop_selection
    :type cl:integer
    :initform 0))
)

(cl:defclass LoopSelection (<LoopSelection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoopSelection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoopSelection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<LoopSelection> is deprecated: use kortex_driver-msg:LoopSelection instead.")))

(cl:ensure-generic-function 'loop_selection-val :lambda-list '(m))
(cl:defmethod loop_selection-val ((m <LoopSelection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:loop_selection-val is deprecated.  Use kortex_driver-msg:loop_selection instead.")
  (loop_selection m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoopSelection>) ostream)
  "Serializes a message object of type '<LoopSelection>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'loop_selection)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'loop_selection)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoopSelection>) istream)
  "Deserializes a message object of type '<LoopSelection>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'loop_selection)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoopSelection>)))
  "Returns string type for a message object of type '<LoopSelection>"
  "kortex_driver/LoopSelection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoopSelection)))
  "Returns string type for a message object of type 'LoopSelection"
  "kortex_driver/LoopSelection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoopSelection>)))
  "Returns md5sum for a message object of type '<LoopSelection>"
  "404b340f5699ef6e44d5690bdce228f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoopSelection)))
  "Returns md5sum for a message object of type 'LoopSelection"
  "404b340f5699ef6e44d5690bdce228f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoopSelection>)))
  "Returns full string definition for message of type '<LoopSelection>"
  (cl:format cl:nil "~%uint32 loop_selection~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoopSelection)))
  "Returns full string definition for message of type 'LoopSelection"
  (cl:format cl:nil "~%uint32 loop_selection~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoopSelection>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoopSelection>))
  "Converts a ROS message object to a list"
  (cl:list 'LoopSelection
    (cl:cons ':loop_selection (loop_selection msg))
))
