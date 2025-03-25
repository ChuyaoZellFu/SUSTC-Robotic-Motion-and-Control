; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetCoggingFeedforwardMode-request.msg.html

(cl:defclass <GetCoggingFeedforwardMode-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetCoggingFeedforwardMode-request (<GetCoggingFeedforwardMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCoggingFeedforwardMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCoggingFeedforwardMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetCoggingFeedforwardMode-request> is deprecated: use kortex_driver-srv:GetCoggingFeedforwardMode-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetCoggingFeedforwardMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCoggingFeedforwardMode-request>) ostream)
  "Serializes a message object of type '<GetCoggingFeedforwardMode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCoggingFeedforwardMode-request>) istream)
  "Deserializes a message object of type '<GetCoggingFeedforwardMode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCoggingFeedforwardMode-request>)))
  "Returns string type for a service object of type '<GetCoggingFeedforwardMode-request>"
  "kortex_driver/GetCoggingFeedforwardModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCoggingFeedforwardMode-request)))
  "Returns string type for a service object of type 'GetCoggingFeedforwardMode-request"
  "kortex_driver/GetCoggingFeedforwardModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCoggingFeedforwardMode-request>)))
  "Returns md5sum for a message object of type '<GetCoggingFeedforwardMode-request>"
  "54f13ab10b17b140d71d14c7e922bc7e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCoggingFeedforwardMode-request)))
  "Returns md5sum for a message object of type 'GetCoggingFeedforwardMode-request"
  "54f13ab10b17b140d71d14c7e922bc7e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCoggingFeedforwardMode-request>)))
  "Returns full string definition for message of type '<GetCoggingFeedforwardMode-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCoggingFeedforwardMode-request)))
  "Returns full string definition for message of type 'GetCoggingFeedforwardMode-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCoggingFeedforwardMode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCoggingFeedforwardMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCoggingFeedforwardMode-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetCoggingFeedforwardMode-response.msg.html

(cl:defclass <GetCoggingFeedforwardMode-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:CoggingFeedforwardModeInformation
    :initform (cl:make-instance 'kortex_driver-msg:CoggingFeedforwardModeInformation)))
)

(cl:defclass GetCoggingFeedforwardMode-response (<GetCoggingFeedforwardMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCoggingFeedforwardMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCoggingFeedforwardMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetCoggingFeedforwardMode-response> is deprecated: use kortex_driver-srv:GetCoggingFeedforwardMode-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetCoggingFeedforwardMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCoggingFeedforwardMode-response>) ostream)
  "Serializes a message object of type '<GetCoggingFeedforwardMode-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCoggingFeedforwardMode-response>) istream)
  "Deserializes a message object of type '<GetCoggingFeedforwardMode-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCoggingFeedforwardMode-response>)))
  "Returns string type for a service object of type '<GetCoggingFeedforwardMode-response>"
  "kortex_driver/GetCoggingFeedforwardModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCoggingFeedforwardMode-response)))
  "Returns string type for a service object of type 'GetCoggingFeedforwardMode-response"
  "kortex_driver/GetCoggingFeedforwardModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCoggingFeedforwardMode-response>)))
  "Returns md5sum for a message object of type '<GetCoggingFeedforwardMode-response>"
  "54f13ab10b17b140d71d14c7e922bc7e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCoggingFeedforwardMode-response)))
  "Returns md5sum for a message object of type 'GetCoggingFeedforwardMode-response"
  "54f13ab10b17b140d71d14c7e922bc7e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCoggingFeedforwardMode-response>)))
  "Returns full string definition for message of type '<GetCoggingFeedforwardMode-response>"
  (cl:format cl:nil "CoggingFeedforwardModeInformation output~%~%================================================================================~%MSG: kortex_driver/CoggingFeedforwardModeInformation~%~%uint32 cogging_feedforward_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCoggingFeedforwardMode-response)))
  "Returns full string definition for message of type 'GetCoggingFeedforwardMode-response"
  (cl:format cl:nil "CoggingFeedforwardModeInformation output~%~%================================================================================~%MSG: kortex_driver/CoggingFeedforwardModeInformation~%~%uint32 cogging_feedforward_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCoggingFeedforwardMode-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCoggingFeedforwardMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCoggingFeedforwardMode-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCoggingFeedforwardMode)))
  'GetCoggingFeedforwardMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCoggingFeedforwardMode)))
  'GetCoggingFeedforwardMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCoggingFeedforwardMode)))
  "Returns string type for a service object of type '<GetCoggingFeedforwardMode>"
  "kortex_driver/GetCoggingFeedforwardMode")