; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude UpdateUserProfile-request.msg.html

(cl:defclass <UpdateUserProfile-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:UserProfile
    :initform (cl:make-instance 'kortex_driver-msg:UserProfile)))
)

(cl:defclass UpdateUserProfile-request (<UpdateUserProfile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateUserProfile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateUserProfile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<UpdateUserProfile-request> is deprecated: use kortex_driver-srv:UpdateUserProfile-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <UpdateUserProfile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateUserProfile-request>) ostream)
  "Serializes a message object of type '<UpdateUserProfile-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateUserProfile-request>) istream)
  "Deserializes a message object of type '<UpdateUserProfile-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateUserProfile-request>)))
  "Returns string type for a service object of type '<UpdateUserProfile-request>"
  "kortex_driver/UpdateUserProfileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateUserProfile-request)))
  "Returns string type for a service object of type 'UpdateUserProfile-request"
  "kortex_driver/UpdateUserProfileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateUserProfile-request>)))
  "Returns md5sum for a message object of type '<UpdateUserProfile-request>"
  "75eb07981bad53b9f3b10bd6121afb95")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateUserProfile-request)))
  "Returns md5sum for a message object of type 'UpdateUserProfile-request"
  "75eb07981bad53b9f3b10bd6121afb95")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateUserProfile-request>)))
  "Returns full string definition for message of type '<UpdateUserProfile-request>"
  (cl:format cl:nil "UserProfile input~%~%================================================================================~%MSG: kortex_driver/UserProfile~%~%UserProfileHandle handle~%string username~%string firstname~%string lastname~%string application_data~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateUserProfile-request)))
  "Returns full string definition for message of type 'UpdateUserProfile-request"
  (cl:format cl:nil "UserProfile input~%~%================================================================================~%MSG: kortex_driver/UserProfile~%~%UserProfileHandle handle~%string username~%string firstname~%string lastname~%string application_data~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateUserProfile-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateUserProfile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateUserProfile-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude UpdateUserProfile-response.msg.html

(cl:defclass <UpdateUserProfile-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass UpdateUserProfile-response (<UpdateUserProfile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateUserProfile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateUserProfile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<UpdateUserProfile-response> is deprecated: use kortex_driver-srv:UpdateUserProfile-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <UpdateUserProfile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateUserProfile-response>) ostream)
  "Serializes a message object of type '<UpdateUserProfile-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateUserProfile-response>) istream)
  "Deserializes a message object of type '<UpdateUserProfile-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateUserProfile-response>)))
  "Returns string type for a service object of type '<UpdateUserProfile-response>"
  "kortex_driver/UpdateUserProfileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateUserProfile-response)))
  "Returns string type for a service object of type 'UpdateUserProfile-response"
  "kortex_driver/UpdateUserProfileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateUserProfile-response>)))
  "Returns md5sum for a message object of type '<UpdateUserProfile-response>"
  "75eb07981bad53b9f3b10bd6121afb95")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateUserProfile-response)))
  "Returns md5sum for a message object of type 'UpdateUserProfile-response"
  "75eb07981bad53b9f3b10bd6121afb95")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateUserProfile-response>)))
  "Returns full string definition for message of type '<UpdateUserProfile-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateUserProfile-response)))
  "Returns full string definition for message of type 'UpdateUserProfile-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateUserProfile-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateUserProfile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateUserProfile-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UpdateUserProfile)))
  'UpdateUserProfile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UpdateUserProfile)))
  'UpdateUserProfile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateUserProfile)))
  "Returns string type for a service object of type '<UpdateUserProfile>"
  "kortex_driver/UpdateUserProfile")