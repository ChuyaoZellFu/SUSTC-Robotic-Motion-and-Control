; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude PasswordChange.msg.html

(cl:defclass <PasswordChange> (roslisp-msg-protocol:ros-message)
  ((handle
    :reader handle
    :initarg :handle
    :type kortex_driver-msg:UserProfileHandle
    :initform (cl:make-instance 'kortex_driver-msg:UserProfileHandle))
   (old_password
    :reader old_password
    :initarg :old_password
    :type cl:string
    :initform "")
   (new_password
    :reader new_password
    :initarg :new_password
    :type cl:string
    :initform ""))
)

(cl:defclass PasswordChange (<PasswordChange>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PasswordChange>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PasswordChange)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<PasswordChange> is deprecated: use kortex_driver-msg:PasswordChange instead.")))

(cl:ensure-generic-function 'handle-val :lambda-list '(m))
(cl:defmethod handle-val ((m <PasswordChange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:handle-val is deprecated.  Use kortex_driver-msg:handle instead.")
  (handle m))

(cl:ensure-generic-function 'old_password-val :lambda-list '(m))
(cl:defmethod old_password-val ((m <PasswordChange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:old_password-val is deprecated.  Use kortex_driver-msg:old_password instead.")
  (old_password m))

(cl:ensure-generic-function 'new_password-val :lambda-list '(m))
(cl:defmethod new_password-val ((m <PasswordChange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:new_password-val is deprecated.  Use kortex_driver-msg:new_password instead.")
  (new_password m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PasswordChange>) ostream)
  "Serializes a message object of type '<PasswordChange>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handle) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'old_password))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'old_password))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'new_password))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'new_password))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PasswordChange>) istream)
  "Deserializes a message object of type '<PasswordChange>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handle) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'old_password) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'old_password) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'new_password) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'new_password) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PasswordChange>)))
  "Returns string type for a message object of type '<PasswordChange>"
  "kortex_driver/PasswordChange")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PasswordChange)))
  "Returns string type for a message object of type 'PasswordChange"
  "kortex_driver/PasswordChange")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PasswordChange>)))
  "Returns md5sum for a message object of type '<PasswordChange>"
  "75d13d1936094613c5e6bc629ee347b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PasswordChange)))
  "Returns md5sum for a message object of type 'PasswordChange"
  "75d13d1936094613c5e6bc629ee347b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PasswordChange>)))
  "Returns full string definition for message of type '<PasswordChange>"
  (cl:format cl:nil "~%UserProfileHandle handle~%string old_password~%string new_password~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PasswordChange)))
  "Returns full string definition for message of type 'PasswordChange"
  (cl:format cl:nil "~%UserProfileHandle handle~%string old_password~%string new_password~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PasswordChange>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handle))
     4 (cl:length (cl:slot-value msg 'old_password))
     4 (cl:length (cl:slot-value msg 'new_password))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PasswordChange>))
  "Converts a ROS message object to a list"
  (cl:list 'PasswordChange
    (cl:cons ':handle (handle msg))
    (cl:cons ':old_password (old_password msg))
    (cl:cons ':new_password (new_password msg))
))
