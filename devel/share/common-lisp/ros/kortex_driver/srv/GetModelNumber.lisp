; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetModelNumber-request.msg.html

(cl:defclass <GetModelNumber-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetModelNumber-request (<GetModelNumber-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetModelNumber-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetModelNumber-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetModelNumber-request> is deprecated: use kortex_driver-srv:GetModelNumber-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetModelNumber-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetModelNumber-request>) ostream)
  "Serializes a message object of type '<GetModelNumber-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetModelNumber-request>) istream)
  "Deserializes a message object of type '<GetModelNumber-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetModelNumber-request>)))
  "Returns string type for a service object of type '<GetModelNumber-request>"
  "kortex_driver/GetModelNumberRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModelNumber-request)))
  "Returns string type for a service object of type 'GetModelNumber-request"
  "kortex_driver/GetModelNumberRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetModelNumber-request>)))
  "Returns md5sum for a message object of type '<GetModelNumber-request>"
  "af635fd95ef990bb7c96e0afad0126ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetModelNumber-request)))
  "Returns md5sum for a message object of type 'GetModelNumber-request"
  "af635fd95ef990bb7c96e0afad0126ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetModelNumber-request>)))
  "Returns full string definition for message of type '<GetModelNumber-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetModelNumber-request)))
  "Returns full string definition for message of type 'GetModelNumber-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetModelNumber-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetModelNumber-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetModelNumber-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetModelNumber-response.msg.html

(cl:defclass <GetModelNumber-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ModelNumber
    :initform (cl:make-instance 'kortex_driver-msg:ModelNumber)))
)

(cl:defclass GetModelNumber-response (<GetModelNumber-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetModelNumber-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetModelNumber-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetModelNumber-response> is deprecated: use kortex_driver-srv:GetModelNumber-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetModelNumber-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetModelNumber-response>) ostream)
  "Serializes a message object of type '<GetModelNumber-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetModelNumber-response>) istream)
  "Deserializes a message object of type '<GetModelNumber-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetModelNumber-response>)))
  "Returns string type for a service object of type '<GetModelNumber-response>"
  "kortex_driver/GetModelNumberResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModelNumber-response)))
  "Returns string type for a service object of type 'GetModelNumber-response"
  "kortex_driver/GetModelNumberResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetModelNumber-response>)))
  "Returns md5sum for a message object of type '<GetModelNumber-response>"
  "af635fd95ef990bb7c96e0afad0126ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetModelNumber-response)))
  "Returns md5sum for a message object of type 'GetModelNumber-response"
  "af635fd95ef990bb7c96e0afad0126ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetModelNumber-response>)))
  "Returns full string definition for message of type '<GetModelNumber-response>"
  (cl:format cl:nil "ModelNumber output~%~%================================================================================~%MSG: kortex_driver/ModelNumber~%~%string model_number~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetModelNumber-response)))
  "Returns full string definition for message of type 'GetModelNumber-response"
  (cl:format cl:nil "ModelNumber output~%~%================================================================================~%MSG: kortex_driver/ModelNumber~%~%string model_number~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetModelNumber-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetModelNumber-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetModelNumber-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetModelNumber)))
  'GetModelNumber-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetModelNumber)))
  'GetModelNumber-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModelNumber)))
  "Returns string type for a service object of type '<GetModelNumber>"
  "kortex_driver/GetModelNumber")