; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ChangePassword-request.msg.html

(cl:defclass <ChangePassword-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:PasswordChange
    :initform (cl:make-instance 'kortex_driver-msg:PasswordChange)))
)

(cl:defclass ChangePassword-request (<ChangePassword-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChangePassword-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChangePassword-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ChangePassword-request> is deprecated: use kortex_driver-srv:ChangePassword-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ChangePassword-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChangePassword-request>) ostream)
  "Serializes a message object of type '<ChangePassword-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChangePassword-request>) istream)
  "Deserializes a message object of type '<ChangePassword-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChangePassword-request>)))
  "Returns string type for a service object of type '<ChangePassword-request>"
  "kortex_driver/ChangePasswordRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangePassword-request)))
  "Returns string type for a service object of type 'ChangePassword-request"
  "kortex_driver/ChangePasswordRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChangePassword-request>)))
  "Returns md5sum for a message object of type '<ChangePassword-request>"
  "df30b453fac13606ed2374a5df7abb80")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChangePassword-request)))
  "Returns md5sum for a message object of type 'ChangePassword-request"
  "df30b453fac13606ed2374a5df7abb80")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChangePassword-request>)))
  "Returns full string definition for message of type '<ChangePassword-request>"
  (cl:format cl:nil "PasswordChange input~%~%================================================================================~%MSG: kortex_driver/PasswordChange~%~%UserProfileHandle handle~%string old_password~%string new_password~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChangePassword-request)))
  "Returns full string definition for message of type 'ChangePassword-request"
  (cl:format cl:nil "PasswordChange input~%~%================================================================================~%MSG: kortex_driver/PasswordChange~%~%UserProfileHandle handle~%string old_password~%string new_password~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChangePassword-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChangePassword-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ChangePassword-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ChangePassword-response.msg.html

(cl:defclass <ChangePassword-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ChangePassword-response (<ChangePassword-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChangePassword-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChangePassword-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ChangePassword-response> is deprecated: use kortex_driver-srv:ChangePassword-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ChangePassword-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChangePassword-response>) ostream)
  "Serializes a message object of type '<ChangePassword-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChangePassword-response>) istream)
  "Deserializes a message object of type '<ChangePassword-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChangePassword-response>)))
  "Returns string type for a service object of type '<ChangePassword-response>"
  "kortex_driver/ChangePasswordResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangePassword-response)))
  "Returns string type for a service object of type 'ChangePassword-response"
  "kortex_driver/ChangePasswordResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChangePassword-response>)))
  "Returns md5sum for a message object of type '<ChangePassword-response>"
  "df30b453fac13606ed2374a5df7abb80")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChangePassword-response)))
  "Returns md5sum for a message object of type 'ChangePassword-response"
  "df30b453fac13606ed2374a5df7abb80")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChangePassword-response>)))
  "Returns full string definition for message of type '<ChangePassword-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChangePassword-response)))
  "Returns full string definition for message of type 'ChangePassword-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChangePassword-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChangePassword-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ChangePassword-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ChangePassword)))
  'ChangePassword-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ChangePassword)))
  'ChangePassword-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangePassword)))
  "Returns string type for a service object of type '<ChangePassword>"
  "kortex_driver/ChangePassword")