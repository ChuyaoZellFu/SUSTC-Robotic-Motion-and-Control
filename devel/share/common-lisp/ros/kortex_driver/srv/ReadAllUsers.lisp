; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ReadAllUsers-request.msg.html

(cl:defclass <ReadAllUsers-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ReadAllUsers-request (<ReadAllUsers-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadAllUsers-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadAllUsers-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ReadAllUsers-request> is deprecated: use kortex_driver-srv:ReadAllUsers-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ReadAllUsers-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadAllUsers-request>) ostream)
  "Serializes a message object of type '<ReadAllUsers-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadAllUsers-request>) istream)
  "Deserializes a message object of type '<ReadAllUsers-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadAllUsers-request>)))
  "Returns string type for a service object of type '<ReadAllUsers-request>"
  "kortex_driver/ReadAllUsersRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadAllUsers-request)))
  "Returns string type for a service object of type 'ReadAllUsers-request"
  "kortex_driver/ReadAllUsersRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadAllUsers-request>)))
  "Returns md5sum for a message object of type '<ReadAllUsers-request>"
  "f4e7c3e24386fc73fa17c3ea4d2a06c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadAllUsers-request)))
  "Returns md5sum for a message object of type 'ReadAllUsers-request"
  "f4e7c3e24386fc73fa17c3ea4d2a06c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadAllUsers-request>)))
  "Returns full string definition for message of type '<ReadAllUsers-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadAllUsers-request)))
  "Returns full string definition for message of type 'ReadAllUsers-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadAllUsers-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadAllUsers-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadAllUsers-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ReadAllUsers-response.msg.html

(cl:defclass <ReadAllUsers-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:UserList
    :initform (cl:make-instance 'kortex_driver-msg:UserList)))
)

(cl:defclass ReadAllUsers-response (<ReadAllUsers-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadAllUsers-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadAllUsers-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ReadAllUsers-response> is deprecated: use kortex_driver-srv:ReadAllUsers-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ReadAllUsers-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadAllUsers-response>) ostream)
  "Serializes a message object of type '<ReadAllUsers-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadAllUsers-response>) istream)
  "Deserializes a message object of type '<ReadAllUsers-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadAllUsers-response>)))
  "Returns string type for a service object of type '<ReadAllUsers-response>"
  "kortex_driver/ReadAllUsersResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadAllUsers-response)))
  "Returns string type for a service object of type 'ReadAllUsers-response"
  "kortex_driver/ReadAllUsersResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadAllUsers-response>)))
  "Returns md5sum for a message object of type '<ReadAllUsers-response>"
  "f4e7c3e24386fc73fa17c3ea4d2a06c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadAllUsers-response)))
  "Returns md5sum for a message object of type 'ReadAllUsers-response"
  "f4e7c3e24386fc73fa17c3ea4d2a06c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadAllUsers-response>)))
  "Returns full string definition for message of type '<ReadAllUsers-response>"
  (cl:format cl:nil "UserList output~%~%================================================================================~%MSG: kortex_driver/UserList~%~%UserProfileHandle[] user_handles~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadAllUsers-response)))
  "Returns full string definition for message of type 'ReadAllUsers-response"
  (cl:format cl:nil "UserList output~%~%================================================================================~%MSG: kortex_driver/UserList~%~%UserProfileHandle[] user_handles~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadAllUsers-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadAllUsers-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadAllUsers-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReadAllUsers)))
  'ReadAllUsers-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReadAllUsers)))
  'ReadAllUsers-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadAllUsers)))
  "Returns string type for a service object of type '<ReadAllUsers>"
  "kortex_driver/ReadAllUsers")