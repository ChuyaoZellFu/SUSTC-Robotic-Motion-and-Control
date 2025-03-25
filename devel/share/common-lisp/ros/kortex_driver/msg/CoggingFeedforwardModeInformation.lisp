; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CoggingFeedforwardModeInformation.msg.html

(cl:defclass <CoggingFeedforwardModeInformation> (roslisp-msg-protocol:ros-message)
  ((cogging_feedforward_mode
    :reader cogging_feedforward_mode
    :initarg :cogging_feedforward_mode
    :type cl:integer
    :initform 0))
)

(cl:defclass CoggingFeedforwardModeInformation (<CoggingFeedforwardModeInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CoggingFeedforwardModeInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CoggingFeedforwardModeInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CoggingFeedforwardModeInformation> is deprecated: use kortex_driver-msg:CoggingFeedforwardModeInformation instead.")))

(cl:ensure-generic-function 'cogging_feedforward_mode-val :lambda-list '(m))
(cl:defmethod cogging_feedforward_mode-val ((m <CoggingFeedforwardModeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:cogging_feedforward_mode-val is deprecated.  Use kortex_driver-msg:cogging_feedforward_mode instead.")
  (cogging_feedforward_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CoggingFeedforwardModeInformation>) ostream)
  "Serializes a message object of type '<CoggingFeedforwardModeInformation>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cogging_feedforward_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cogging_feedforward_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cogging_feedforward_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cogging_feedforward_mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CoggingFeedforwardModeInformation>) istream)
  "Deserializes a message object of type '<CoggingFeedforwardModeInformation>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cogging_feedforward_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cogging_feedforward_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cogging_feedforward_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cogging_feedforward_mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CoggingFeedforwardModeInformation>)))
  "Returns string type for a message object of type '<CoggingFeedforwardModeInformation>"
  "kortex_driver/CoggingFeedforwardModeInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CoggingFeedforwardModeInformation)))
  "Returns string type for a message object of type 'CoggingFeedforwardModeInformation"
  "kortex_driver/CoggingFeedforwardModeInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CoggingFeedforwardModeInformation>)))
  "Returns md5sum for a message object of type '<CoggingFeedforwardModeInformation>"
  "e26fc90c6a8fc2a4d783f21f241b9871")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CoggingFeedforwardModeInformation)))
  "Returns md5sum for a message object of type 'CoggingFeedforwardModeInformation"
  "e26fc90c6a8fc2a4d783f21f241b9871")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CoggingFeedforwardModeInformation>)))
  "Returns full string definition for message of type '<CoggingFeedforwardModeInformation>"
  (cl:format cl:nil "~%uint32 cogging_feedforward_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CoggingFeedforwardModeInformation)))
  "Returns full string definition for message of type 'CoggingFeedforwardModeInformation"
  (cl:format cl:nil "~%uint32 cogging_feedforward_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CoggingFeedforwardModeInformation>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CoggingFeedforwardModeInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'CoggingFeedforwardModeInformation
    (cl:cons ':cogging_feedforward_mode (cogging_feedforward_mode msg))
))
