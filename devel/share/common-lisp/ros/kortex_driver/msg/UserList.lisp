; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude UserList.msg.html

(cl:defclass <UserList> (roslisp-msg-protocol:ros-message)
  ((user_handles
    :reader user_handles
    :initarg :user_handles
    :type (cl:vector kortex_driver-msg:UserProfileHandle)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:UserProfileHandle :initial-element (cl:make-instance 'kortex_driver-msg:UserProfileHandle))))
)

(cl:defclass UserList (<UserList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UserList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UserList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<UserList> is deprecated: use kortex_driver-msg:UserList instead.")))

(cl:ensure-generic-function 'user_handles-val :lambda-list '(m))
(cl:defmethod user_handles-val ((m <UserList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:user_handles-val is deprecated.  Use kortex_driver-msg:user_handles instead.")
  (user_handles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UserList>) ostream)
  "Serializes a message object of type '<UserList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'user_handles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'user_handles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UserList>) istream)
  "Deserializes a message object of type '<UserList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'user_handles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'user_handles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:UserProfileHandle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UserList>)))
  "Returns string type for a message object of type '<UserList>"
  "kortex_driver/UserList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UserList)))
  "Returns string type for a message object of type 'UserList"
  "kortex_driver/UserList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UserList>)))
  "Returns md5sum for a message object of type '<UserList>"
  "fd1c8a5c2edbaa168a73081302b5bac5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UserList)))
  "Returns md5sum for a message object of type 'UserList"
  "fd1c8a5c2edbaa168a73081302b5bac5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UserList>)))
  "Returns full string definition for message of type '<UserList>"
  (cl:format cl:nil "~%UserProfileHandle[] user_handles~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UserList)))
  "Returns full string definition for message of type 'UserList"
  (cl:format cl:nil "~%UserProfileHandle[] user_handles~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UserList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'user_handles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UserList>))
  "Converts a ROS message object to a list"
  (cl:list 'UserList
    (cl:cons ':user_handles (user_handles msg))
))
