import axios from 'axios';

const API_BASE_URL = import.meta.env.VITE_API_BASE_URL || '/api';

export interface Message {
  id: string;
  conversationId: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: string;
  metadata?: {
    model: string;
    tokens: number;
    latency: number;
  };
}

export interface Conversation {
  id: string;
  createdAt: string;
  updatedAt: string;
  messages: Message[];
  context: {
    summary: string;
    topics: string[];
    tokenCount: number;
  };
  status: string;
}

export interface CreateConversationResponse {
  id: string;
  createdAt: string;
  status: string;
  messages: Message[];
}

export interface SendMessageRequest {
  content: string;
}

class ChatApi {
  private axiosInstance = axios.create({
    baseURL: API_BASE_URL,
    headers: {
      'Content-Type': 'application/json',
    },
  });

  async createConversation(): Promise<CreateConversationResponse> {
    const response = await this.axiosInstance.post<CreateConversationResponse>(
      '/chat/conversations'
    );
    return response.data;
  }

  async sendMessage(
    conversationId: string,
    content: string
  ): Promise<Message> {
    const response = await this.axiosInstance.post<Message>(
      `/chat/conversations/${conversationId}/messages`,
      { content }
    );
    return response.data;
  }

  async getConversation(conversationId: string): Promise<Conversation> {
    const response = await this.axiosInstance.get<Conversation>(
      `/chat/conversations/${conversationId}`
    );
    return response.data;
  }

  async endConversation(conversationId: string): Promise<void> {
    await this.axiosInstance.delete(
      `/chat/conversations/${conversationId}`
    );
  }

  async healthCheck(): Promise<{ status: string }> {
    const response = await this.axiosInstance.get('/health');
    return response.data;
  }
}

export const chatApi = new ChatApi();
