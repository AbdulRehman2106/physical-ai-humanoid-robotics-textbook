import axios, { AxiosInstance } from 'axios';

export interface ChatQuery {
  query: string;
  conversation_id?: string;
  filters?: Record<string, any>;
}

export interface ChatQueryWithContext extends ChatQuery {
  selected_text?: string;
  selection_metadata?: {
    chapter_title?: string;
    section_title?: string;
    url?: string;
  };
}

export interface SourceReference {
  chapter_number: number;
  chapter_title: string;
  section_title: string;
  url: string;
  relevance_score: number;
}

export interface ChatResponse {
  answer: string;
  sources: SourceReference[];
  conversation_id: string;
  message_id: string;
}

export interface Message {
  id: string;
  conversation_id: string;
  role: 'user' | 'assistant';
  content: string;
  context_used?: string[];
  created_at: string;
  metadata?: Record<string, any>;
}

export interface Conversation {
  id: string;
  created_at: string;
  metadata?: Record<string, any>;
  messages?: Message[];
}

class ChatAPI {
  private client: AxiosInstance;

  constructor() {
    // Use window object to safely access environment variables in browser
    const baseURL = (typeof window !== 'undefined' && (window as any).BACKEND_URL)
      || 'http://localhost:8001';

    this.client = axios.create({
      baseURL,
      headers: {
        'Content-Type': 'application/json',
      },
      timeout: 60000, // Increased to 60 seconds for RAG queries
    });
  }

  async query(request: ChatQuery): Promise<ChatResponse> {
    const response = await this.client.post<ChatResponse>('/api/chat/query', request);
    return response.data;
  }

  async queryWithContext(request: ChatQueryWithContext): Promise<ChatResponse> {
    const response = await this.client.post<ChatResponse>('/api/chat/query-with-context', request);
    return response.data;
  }

  async createConversation(): Promise<{ conversation_id: string }> {
    const response = await this.client.post<{ conversation_id: string }>('/api/chat/conversations');
    return response.data;
  }

  async getConversation(conversationId: string): Promise<Conversation> {
    const response = await this.client.get<Conversation>(`/api/chat/conversations/${conversationId}`);
    return response.data;
  }

  async healthCheck(): Promise<{ status: string; service: string }> {
    const response = await this.client.get('/api/health');
    return response.data;
  }
}

export const chatAPI = new ChatAPI();
