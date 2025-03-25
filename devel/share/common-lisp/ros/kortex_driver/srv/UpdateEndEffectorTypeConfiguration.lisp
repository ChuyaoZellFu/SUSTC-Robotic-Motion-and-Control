; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude UpdateEndEffectorTypeConfiguration-request.msg.html

(cl:defclass <UpdateEndEffectorTypeConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ProductConfigurationEndEffectorType
    :initform (cl:make-instance 'kortex_driver-msg:ProductConfigurationEndEffectorType)))
)

(cl:defclass UpdateEndEffectorTypeConfiguration-request (<UpdateEndEffectorTypeConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateEndEffectorTypeConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateEndEffectorTypeConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<UpdateEndEffectorTypeConfiguration-request> is deprecated: use kortex_driver-srv:UpdateEndEffectorTypeConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <UpdateEndEffectorTypeConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateEndEffectorTypeConfiguration-request>) ostream)
  "Serializes a message object of type '<UpdateEndEffectorTypeConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateEndEffectorTypeConfiguration-request>) istream)
  "Deserializes a message object of type '<UpdateEndEffectorTypeConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateEndEffectorTypeConfiguration-request>)))
  "Returns string type for a service object of type '<UpdateEndEffectorTypeConfiguration-request>"
  "kortex_driver/UpdateEndEffectorTypeConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateEndEffectorTypeConfiguration-request)))
  "Returns string type for a service object of type 'UpdateEndEffectorTypeConfiguration-request"
  "kortex_driver/UpdateEndEffectorTypeConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateEndEffectorTypeConfiguration-request>)))
  "Returns md5sum for a message object of type '<UpdateEndEffectorTypeConfiguration-request>"
  "f022a6fb3cba8e0906c5a35f2f76f468")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateEndEffectorTypeConfiguration-request)))
  "Returns md5sum for a message object of type 'UpdateEndEffectorTypeConfiguration-request"
  "f022a6fb3cba8e0906c5a35f2f76f468")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateEndEffectorTypeConfiguration-request>)))
  "Returns full string definition for message of type '<UpdateEndEffectorTypeConfiguration-request>"
  (cl:format cl:nil "ProductConfigurationEndEffectorType input~%~%================================================================================~%MSG: kortex_driver/ProductConfigurationEndEffectorType~%~%uint32 end_effector_type~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateEndEffectorTypeConfiguration-request)))
  "Returns full string definition for message of type 'UpdateEndEffectorTypeConfiguration-request"
  (cl:format cl:nil "ProductConfigurationEndEffectorType input~%~%================================================================================~%MSG: kortex_driver/ProductConfigurationEndEffectorType~%~%uint32 end_effector_type~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateEndEffectorTypeConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateEndEffectorTypeConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateEndEffectorTypeConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude UpdateEndEffectorTypeConfiguration-response.msg.html

(cl:defclass <UpdateEndEffectorTypeConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass UpdateEndEffectorTypeConfiguration-response (<UpdateEndEffectorTypeConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateEndEffectorTypeConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateEndEffectorTypeConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<UpdateEndEffectorTypeConfiguration-response> is deprecated: use kortex_driver-srv:UpdateEndEffectorTypeConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <UpdateEndEffectorTypeConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateEndEffectorTypeConfiguration-response>) ostream)
  "Serializes a message object of type '<UpdateEndEffectorTypeConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateEndEffectorTypeConfiguration-response>) istream)
  "Deserializes a message object of type '<UpdateEndEffectorTypeConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateEndEffectorTypeConfiguration-response>)))
  "Returns string type for a service object of type '<UpdateEndEffectorTypeConfiguration-response>"
  "kortex_driver/UpdateEndEffectorTypeConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateEndEffectorTypeConfiguration-response)))
  "Returns string type for a service object of type 'UpdateEndEffectorTypeConfiguration-response"
  "kortex_driver/UpdateEndEffectorTypeConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateEndEffectorTypeConfiguration-response>)))
  "Returns md5sum for a message object of type '<UpdateEndEffectorTypeConfiguration-response>"
  "f022a6fb3cba8e0906c5a35f2f76f468")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateEndEffectorTypeConfiguration-response)))
  "Returns md5sum for a message object of type 'UpdateEndEffectorTypeConfiguration-response"
  "f022a6fb3cba8e0906c5a35f2f76f468")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateEndEffectorTypeConfiguration-response>)))
  "Returns full string definition for message of type '<UpdateEndEffectorTypeConfiguration-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateEndEffectorTypeConfiguration-response)))
  "Returns full string definition for message of type 'UpdateEndEffectorTypeConfiguration-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateEndEffectorTypeConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateEndEffectorTypeConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateEndEffectorTypeConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UpdateEndEffectorTypeConfiguration)))
  'UpdateEndEffectorTypeConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UpdateEndEffectorTypeConfiguration)))
  'UpdateEndEffectorTypeConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateEndEffectorTypeConfiguration)))
  "Returns string type for a service object of type '<UpdateEndEffectorTypeConfiguration>"
  "kortex_driver/UpdateEndEffectorTypeConfiguration")