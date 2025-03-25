; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetCoggingFeedforwardMode-request.msg.html

(cl:defclass <SetCoggingFeedforwardMode-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:CoggingFeedforwardModeInformation
    :initform (cl:make-instance 'kortex_driver-msg:CoggingFeedforwardModeInformation)))
)

(cl:defclass SetCoggingFeedforwardMode-request (<SetCoggingFeedforwardMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCoggingFeedforwardMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCoggingFeedforwardMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetCoggingFeedforwardMode-request> is deprecated: use kortex_driver-srv:SetCoggingFeedforwardMode-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetCoggingFeedforwardMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCoggingFeedforwardMode-request>) ostream)
  "Serializes a message object of type '<SetCoggingFeedforwardMode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCoggingFeedforwardMode-request>) istream)
  "Deserializes a message object of type '<SetCoggingFeedforwardMode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCoggingFeedforwardMode-request>)))
  "Returns string type for a service object of type '<SetCoggingFeedforwardMode-request>"
  "kortex_driver/SetCoggingFeedforwardModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCoggingFeedforwardMode-request)))
  "Returns string type for a service object of type 'SetCoggingFeedforwardMode-request"
  "kortex_driver/SetCoggingFeedforwardModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCoggingFeedforwardMode-request>)))
  "Returns md5sum for a message object of type '<SetCoggingFeedforwardMode-request>"
  "156e5a43117ffd12af553796e457d579")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCoggingFeedforwardMode-request)))
  "Returns md5sum for a message object of type 'SetCoggingFeedforwardMode-request"
  "156e5a43117ffd12af553796e457d579")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCoggingFeedforwardMode-request>)))
  "Returns full string definition for message of type '<SetCoggingFeedforwardMode-request>"
  (cl:format cl:nil "CoggingFeedforwardModeInformation input~%~%================================================================================~%MSG: kortex_driver/CoggingFeedforwardModeInformation~%~%uint32 cogging_feedforward_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCoggingFeedforwardMode-request)))
  "Returns full string definition for message of type 'SetCoggingFeedforwardMode-request"
  (cl:format cl:nil "CoggingFeedforwardModeInformation input~%~%================================================================================~%MSG: kortex_driver/CoggingFeedforwardModeInformation~%~%uint32 cogging_feedforward_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCoggingFeedforwardMode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCoggingFeedforwardMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCoggingFeedforwardMode-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetCoggingFeedforwardMode-response.msg.html

(cl:defclass <SetCoggingFeedforwardMode-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetCoggingFeedforwardMode-response (<SetCoggingFeedforwardMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCoggingFeedforwardMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCoggingFeedforwardMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetCoggingFeedforwardMode-response> is deprecated: use kortex_driver-srv:SetCoggingFeedforwardMode-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetCoggingFeedforwardMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCoggingFeedforwardMode-response>) ostream)
  "Serializes a message object of type '<SetCoggingFeedforwardMode-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCoggingFeedforwardMode-response>) istream)
  "Deserializes a message object of type '<SetCoggingFeedforwardMode-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCoggingFeedforwardMode-response>)))
  "Returns string type for a service object of type '<SetCoggingFeedforwardMode-response>"
  "kortex_driver/SetCoggingFeedforwardModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCoggingFeedforwardMode-response)))
  "Returns string type for a service object of type 'SetCoggingFeedforwardMode-response"
  "kortex_driver/SetCoggingFeedforwardModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCoggingFeedforwardMode-response>)))
  "Returns md5sum for a message object of type '<SetCoggingFeedforwardMode-response>"
  "156e5a43117ffd12af553796e457d579")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCoggingFeedforwardMode-response)))
  "Returns md5sum for a message object of type 'SetCoggingFeedforwardMode-response"
  "156e5a43117ffd12af553796e457d579")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCoggingFeedforwardMode-response>)))
  "Returns full string definition for message of type '<SetCoggingFeedforwardMode-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCoggingFeedforwardMode-response)))
  "Returns full string definition for message of type 'SetCoggingFeedforwardMode-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCoggingFeedforwardMode-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCoggingFeedforwardMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCoggingFeedforwardMode-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetCoggingFeedforwardMode)))
  'SetCoggingFeedforwardMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetCoggingFeedforwardMode)))
  'SetCoggingFeedforwardMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCoggingFeedforwardMode)))
  "Returns string type for a service object of type '<SetCoggingFeedforwardMode>"
  "kortex_driver/SetCoggingFeedforwardMode")