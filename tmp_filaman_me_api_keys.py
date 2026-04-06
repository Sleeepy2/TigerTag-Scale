from fastapi import APIRouter, HTTPException, Response, status
from pydantic import BaseModel
from sqlalchemy import select

from app.api.deps import DBSession, PrincipalDep
from app.core.security import generate_token_secret, hash_token
from app.models import UserApiKey

router = APIRouter(prefix="/me/api-keys", tags=["me"])


class CreateApiKeyRequest(BaseModel):
    name: str


class ApiKeyCreatedResponse(BaseModel):
    id: int
    name: str
    token: str
    created_at: str

    class Config:
        from_attributes = True


class ApiKeyListItem(BaseModel):
    id: int
    name: str
    created_at: str
    last_used_at: str | None

    class Config:
        from_attributes = True


@router.post("", response_model=ApiKeyCreatedResponse, status_code=status.HTTP_201_CREATED)
async def create_api_key(
    data: CreateApiKeyRequest,
    principal: PrincipalDep,
    db: DBSession,
):
    secret = generate_token_secret()
    key_hash = hash_token(secret)

    api_key = UserApiKey(
        user_id=principal.user_id,
        name=data.name,
        key_hash=key_hash,
        scopes=None,
    )
    db.add(api_key)
    await db.commit()
    await db.refresh(api_key)

    token = f"uak.{api_key.id}.{secret}"

    return ApiKeyCreatedResponse(
        id=api_key.id,
        name=api_key.name,
        token=token,
        created_at=api_key.created_at.isoformat(),
    )


@router.get("", response_model=list[ApiKeyListItem])
async def list_api_keys(
    principal: PrincipalDep,
    db: DBSession,
):
    result = await db.execute(select(UserApiKey).where(UserApiKey.user_id == principal.user_id))
    keys = result.scalars().all()

    return [
        ApiKeyListItem(
            id=key.id,
            name=key.name,
            created_at=key.created_at.isoformat(),
            last_used_at=key.last_used_at.isoformat() if key.last_used_at else None,
        )
        for key in keys
    ]


@router.delete("/{key_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_api_key(
    key_id: int,
    principal: PrincipalDep,
    db: DBSession,
):
    result = await db.execute(
        select(UserApiKey).where(
            UserApiKey.user_id == principal.user_id,
            UserApiKey.id == key_id,
        )
    )
    api_key = result.scalar_one_or_none()

    if not api_key:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={"code": "not_found", "message": "API key not found"},
        )

    await db.delete(api_key)
    await db.commit()

    return Response(status_code=status.HTTP_204_NO_CONTENT)

