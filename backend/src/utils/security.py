from datetime import datetime, timedelta
from typing import Optional
import secrets
import hashlib
from passlib.context import CryptContext
from jose import JWTError, jwt
from ..config.settings import settings


# Password hashing context with fallback - wrap in function to avoid import-time errors
def _initialize_password_context():
    try:
        pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
        # Test if bcrypt is working properly by hashing a simple password
        test_hash = pwd_context.hash("test")
        # If we get here, bcrypt is working
        return pwd_context
    except Exception as e:
        # If bcrypt fails, use argon2 as fallback
        try:
            pwd_context = CryptContext(schemes=["argon2"], deprecated="auto")
            # Test if argon2 is working properly
            test_hash = pwd_context.hash("test")
            # If we get here, argon2 is working
            return pwd_context
        except Exception:
            # If both fail, use a simple fallback
            import hashlib
            import secrets

            def simple_hash_password(password: str) -> str:
                """Simple password hashing as fallback."""
                salt = secrets.token_hex(16)
                pwdhash = hashlib.pbkdf2_hmac('sha256',
                                             password.encode('utf-8'),
                                             salt.encode('utf-8'),
                                             100000)
                return salt + pwdhash.hex()

            def simple_verify_password(plain_password: str, hashed_password: str) -> bool:
                """Simple password verification as fallback."""
                if len(hashed_password) < 32:
                    return False
                salt = hashed_password[:32]
                stored_password = hashed_password[32:]
                pwdhash = hashlib.pbkdf2_hmac('sha256',
                                             plain_password.encode('utf-8'),
                                             salt.encode('utf-8'),
                                             100000)
                return pwdhash.hex() == stored_password

            # Set up a simple context-like interface
            class SimpleContext:
                def hash(self, secret):
                    return simple_hash_password(secret)

                def verify(self, secret, hash):
                    return simple_verify_password(secret, hash)

            return SimpleContext()

# Initialize the password context
pwd_context = _initialize_password_context()


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a plain password against its hash."""
    # Bcrypt has a 72-byte password length limit, so we truncate if necessary
    if len(plain_password.encode('utf-8')) > 72:
        plain_password = plain_password.encode('utf-8')[:72].decode('utf-8', errors='ignore')
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password: str) -> str:
    """Generate a hash for the given password."""
    # Bcrypt has a 72-byte password length limit, so we truncate if necessary
    if len(password.encode('utf-8')) > 72:
        password = password.encode('utf-8')[:72].decode('utf-8', errors='ignore')
    return pwd_context.hash(password)


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    """Create a JWT access token."""
    to_encode = data.copy()

    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)

    to_encode.update({"exp": expire, "type": "access"})
    encoded_jwt = jwt.encode(to_encode, settings.JWT_SECRET_KEY, algorithm=settings.JWT_ALGORITHM)
    return encoded_jwt


def create_refresh_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    """Create a JWT refresh token."""
    to_encode = data.copy()

    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(days=settings.REFRESH_TOKEN_EXPIRE_DAYS)

    to_encode.update({"exp": expire, "type": "refresh"})
    encoded_jwt = jwt.encode(to_encode, settings.JWT_SECRET_KEY, algorithm=settings.JWT_ALGORITHM)
    return encoded_jwt


def verify_token(token: str) -> Optional[dict]:
    """Verify a JWT token and return its payload if valid."""
    try:
        payload = jwt.decode(token, settings.JWT_SECRET_KEY, algorithms=[settings.JWT_ALGORITHM])
        return payload
    except JWTError:
        return None


def generate_secret_key(length: int = 32) -> str:
    """Generate a random secret key."""
    return secrets.token_urlsafe(length)


def hash_data(data: str) -> str:
    """Create a SHA256 hash of the given data."""
    return hashlib.sha256(data.encode()).hexdigest()


def generate_api_key() -> str:
    """Generate a random API key."""
    return secrets.token_urlsafe(32)