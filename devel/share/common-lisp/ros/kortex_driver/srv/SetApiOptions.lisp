; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetApiOptions-request.msg.html

(cl:defclass <SetApiOptions-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ApiOptions
    :initform (cl:make-instance 'kortex_driver-msg:ApiOptions)))
)

(cl:defclass SetApiOptions-request (<SetApiOptions-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetApiOptions-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetApiOptions-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetApiOptions-request> is deprecated: use kortex_driver-srv:SetApiOptions-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetApiOptions-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetApiOptions-request>) ostream)
  "Serializes a message object of type '<SetApiOptions-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetApiOptions-request>) istream)
  "Deserializes a message object of type '<SetApiOptions-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetApiOptions-request>)))
  "Returns string type for a service object of type '<SetApiOptions-request>"
  "kortex_driver/SetApiOptionsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetApiOptions-request)))
  "Returns string type for a service object of type 'SetApiOptions-request"
  "kortex_driver/SetApiOptionsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetApiOptions-request>)))
  "Returns md5sum for a message object of type '<SetApiOptions-request>"
  "11c85b567b00c6eb6cd55fdb8cb9ad1b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetApiOptions-request)))
  "Returns md5sum for a message object of type 'SetApiOptions-request"
  "11c85b567b00c6eb6cd55fdb8cb9ad1b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetApiOptions-request>)))
  "Returns full string definition for message of type '<SetApiOptions-request>"
  (cl:format cl:nil "ApiOptions input~%~%================================================================================~%MSG: kortex_driver/ApiOptions~%uint32 timeout_ms~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetApiOptions-request)))
  "Returns full string definition for message of type 'SetApiOptions-request"
  (cl:format cl:nil "ApiOptions input~%~%================================================================================~%MSG: kortex_driver/ApiOptions~%uint32 timeout_ms~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetApiOptions-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetApiOptions-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetApiOptions-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetApiOptions-response.msg.html

(cl:defclass <SetApiOptions-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetApiOptions-response (<SetApiOptions-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetApiOptions-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetApiOptions-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetApiOptions-response> is deprecated: use kortex_driver-srv:SetApiOptions-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetApiOptions-response>) ostream)
  "Serializes a message object of type '<SetApiOptions-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetApiOptions-response>) istream)
  "Deserializes a message object of type '<SetApiOptions-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetApiOptions-response>)))
  "Returns string type for a service object of type '<SetApiOptions-response>"
  "kortex_driver/SetApiOptionsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetApiOptions-response)))
  "Returns string type for a service object of type 'SetApiOptions-response"
  "kortex_driver/SetApiOptionsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetApiOptions-response>)))
  "Returns md5sum for a message object of type '<SetApiOptions-response>"
  "11c85b567b00c6eb6cd55fdb8cb9ad1b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetApiOptions-response)))
  "Returns md5sum for a message object of type 'SetApiOptions-response"
  "11c85b567b00c6eb6cd55fdb8cb9ad1b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetApiOptions-response>)))
  "Returns full string definition for message of type '<SetApiOptions-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetApiOptions-response)))
  "Returns full string definition for message of type 'SetApiOptions-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetApiOptions-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetApiOptions-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetApiOptions-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetApiOptions)))
  'SetApiOptions-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetApiOptions)))
  'SetApiOptions-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetApiOptions)))
  "Returns string type for a service object of type '<SetApiOptions>"
  "kortex_driver/SetApiOptions")