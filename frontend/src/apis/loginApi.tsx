import axios from 'axios';
import {LoginRequestPayload} from '../types/LoginRequestPayload';
import {LoginResponsePayload} from '../types/LoginResponsePayload';

const API_URL = 'http://api.example.com'; // API 기본 URL 설정

export const loginApi = async (
  payload: LoginRequestPayload,
): Promise<LoginResponsePayload> => {
  const response = await axios.post(`${API_URL}/login`, payload);
  return response.data;
};
