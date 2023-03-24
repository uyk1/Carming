import {createAsyncThunk, createSlice, PayloadAction} from '@reduxjs/toolkit';
import {Member} from '../../types';
import {LoginRequestPayload} from '../../types/LoginRequestPayload';
import {LoginResponsePayload} from '../../types/LoginResponsePayload';
import {loginApi} from './../../apis/loginApi';

type AuthState = {
  token: string | null;
  member: Member | null;
  isLoading: boolean;
  error: string | null;
  isLoggedIn: boolean;
};

const initialState: AuthState = {
  token: null,
  member: null,
  isLoading: false,
  error: null,
  isLoggedIn: false,
};

// 로그인 API 호출 액션 생성
export const login = createAsyncThunk(
  'auth/login',
  async (payload: LoginRequestPayload) => {
    const response = await loginApi(payload);
    return response;
  },
);

const authSlice = createSlice({
  name: 'auth',
  initialState,
  reducers: {
    loginStart: state => {
      state.isLoading = true;
      state.error = null;
    },
    loginSuccess: (state, action: PayloadAction<LoginResponsePayload>) => {
      const {token, member} = action.payload;
      state.token = token;
      state.member = member;
      state.isLoading = false;
      state.error = null;
      state.isLoggedIn = true;
    },
    loginFailure: (state, action: PayloadAction<string>) => {
      state.isLoading = false;
      state.error = action.payload;
    },
    logout: state => {
      state.token = null;
      state.member = null;
      state.isLoading = false;
      state.error = null;
      state.isLoggedIn = false;
    },
  },
  extraReducers: builder => {
    builder
      .addCase(login.pending, state => {
        state.isLoading = true;
        state.error = null;
      })
      .addCase(
        login.fulfilled,
        (state, action: PayloadAction<LoginResponsePayload>) => {
          const {token, member} = action.payload;
          state.token = token;
          state.member = member;
          state.isLoading = false;
          state.error = null;
          state.isLoggedIn = true;
        },
      )
      .addCase(login.rejected, (state, action) => {
        state.isLoading = false;
        state.error = action.error.message ?? '로그인 중 오류가 발생했습니다.';
      });
  },
});

export const {loginStart, loginSuccess, loginFailure, logout} =
  authSlice.actions;

export default authSlice;
