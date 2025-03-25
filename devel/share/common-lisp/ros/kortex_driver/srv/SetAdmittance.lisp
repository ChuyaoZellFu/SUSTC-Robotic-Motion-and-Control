; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetAdmittance-request.msg.html

(cl:defclass <SetAdmittance-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Admittance
    :initform (cl:make-instance 'kortex_driver-msg:Admittance)))
)

(cl:defclass SetAdmittance-request (<SetAdmittance-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAdmittance-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAdmittance-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetAdmittance-request> is deprecated: use kortex_driver-srv:SetAdmittance-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetAdmittance-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAdmittance-request>) ostream)
  "Serializes a message object of type '<SetAdmittance-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAdmittance-request>) istream)
  "Deserializes a message object of type '<SetAdmittance-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAdmittance-request>)))
  "Returns string type for a service object of type '<SetAdmittance-request>"
  "kortex_driver/SetAdmittanceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAdmittance-request)))
  "Returns string type for a service object of type 'SetAdmittance-request"
  "kortex_driver/SetAdmittanceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAdmittance-request>)))
  "Returns md5sum for a message object of type '<SetAdmittance-request>"
  "3af9f43d4260b726067035bc086c02b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAdmittance-request)))
  "Returns md5sum for a message object of type 'SetAdmittance-request"
  "3af9f43d4260b726067035bc086c02b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAdmittance-request>)))
  "Returns full string definition for message of type '<SetAdmittance-request>"
  (cl:format cl:nil "Admittance input~%~%================================================================================~%MSG: kortex_driver/Admittance~%~%uint32 admittance_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAdmittance-request)))
  "Returns full string definition for message of type 'SetAdmittance-request"
  (cl:format cl:nil "Admittance input~%~%================================================================================~%MSG: kortex_driver/Admittance~%~%uint32 admittance_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAdmittance-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAdmittance-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAdmittance-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetAdmittance-response.msg.html

(cl:defclass <SetAdmittance-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetAdmittance-response (<SetAdmittance-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAdmittance-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAdmittance-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetAdmittance-response> is deprecated: use kortex_driver-srv:SetAdmittance-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetAdmittance-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAdmittance-response>) ostream)
  "Serializes a message object of type '<SetAdmittance-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAdmittance-response>) istream)
  "Deserializes a message object of type '<SetAdmittance-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAdmittance-response>)))
  "Returns string type for a service object of type '<SetAdmittance-response>"
  "kortex_driver/SetAdmittanceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAdmittance-response)))
  "Returns string type for a service object of type 'SetAdmittance-response"
  "kortex_driver/SetAdmittanceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAdmittance-response>)))
  "Returns md5sum for a message object of type '<SetAdmittance-response>"
  "3af9f43d4260b726067035bc086c02b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAdmittance-response)))
  "Returns md5sum for a message object of type 'SetAdmittance-response"
  "3af9f43d4260b726067035bc086c02b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAdmittance-response>)))
  "Returns full string definition for message of type '<SetAdmittance-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAdmittance-response)))
  "Returns full string definition for message of type 'SetAdmittance-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAdmittance-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAdmittance-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAdmittance-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetAdmittance)))
  'SetAdmittance-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetAdmittance)))
  'SetAdmittance-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAdmittance)))
  "Returns string type for a service object of type '<SetAdmittance>"
  "kortex_driver/SetAdmittance")