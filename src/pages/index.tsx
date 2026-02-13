import React, { useEffect, useRef, useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';
import { useCountUp } from '../hooks/useCountUp';
import { FeatureCard, ScrollIndicator, SearchBar } from '../components';
import ChatBot from '../components/ChatBot';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning 📚
          </Link>
          <Link
            className="button button--outline button--secondary button--lg"
            to="/docs/chapters/physical-ai-intro/">
            Jump to Chapter 1 →
          </Link>
        </div>
        <div style={{ marginTop: '2rem', display: 'flex', justifyContent: 'center' }}>
          <SearchBar />
        </div>
      </div>
      <ScrollIndicator />
    </header>
  );
}

function Feature({title, description, icon, gradient, delay}) {
  return (
    <div className={clsx('col col--4')}>
      <FeatureCard
        icon={icon}
        title={title}
        description={description}
        gradient={gradient}
        delay={delay}
      />
    </div>
  );
}

function HomepageFeatures() {
  const features = [
    {
      icon: '🤖',
      title: 'Complete Curriculum',
      description: '11 comprehensive chapters covering Physical AI from fundamentals to advanced deployment',
      gradient: 'linear-gradient(135deg, var(--ifm-color-primary) 0%, var(--ifm-color-primary-dark) 100%)',
      delay: 0.1,
    },
    {
      icon: '💻',
      title: 'Hands-On Learning',
      description: '20+ code examples, interactive quizzes, and a complete capstone project',
      gradient: 'linear-gradient(135deg, #f093fb 0%, #f5576c 100%)',
      delay: 0.2,
    },
    {
      icon: '🎯',
      title: 'Industry-Ready',
      description: 'Learn ROS 2, Gazebo, Isaac Sim, VLA models, and production best practices',
      gradient: 'linear-gradient(135deg, #4facfe 0%, #00f2fe 100%)',
      delay: 0.3,
    },
    {
      icon: '📚',
      title: 'Theory + Practice',
      description: 'Balanced approach with theoretical foundations and practical implementation',
      gradient: 'linear-gradient(135deg, #43e97b 0%, #38f9d7 100%)',
      delay: 0.4,
    },
    {
      icon: '🧠',
      title: 'Cutting-Edge AI',
      description: 'Vision-Language-Action models, sim-to-real transfer, and embodied intelligence',
      gradient: 'linear-gradient(135deg, #fa709a 0%, #fee140 100%)',
      delay: 0.5,
    },
    {
      icon: '🚀',
      title: 'Career-Focused',
      description: 'Build skills for robotics engineering, AI research, and autonomous systems',
      gradient: 'linear-gradient(135deg, #30cfd0 0%, #330867 100%)',
      delay: 0.6,
    },
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <h2 className="text--center margin-bottom--xl" style={{ fontSize: '2.5rem', fontWeight: 700 }}>
          ✨ Why Choose This Textbook?
        </h2>
        <div className="row">
          {features.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

function ChapterOverview() {
  const chapters = [
    {
      title: 'Part 1: Fundamentals',
      items: ['Introduction to Physical AI', 'Embodied Intelligence & Reality Gap'],
    },
    {
      title: 'Part 2: ROS 2 Development',
      items: ['ROS 2 Fundamentals', 'Communication Patterns'],
    },
    {
      title: 'Part 3: Simulation',
      items: ['Simulation Intro', 'Gazebo Basics', 'NVIDIA Isaac Sim'],
    },
    {
      title: 'Part 4: Advanced AI',
      items: ['Vision-Language-Action Models', 'Sim-to-Real Transfer'],
    },
    {
      title: 'Part 5: Production',
      items: ['Error Handling & Robustness', 'Capstone Project'],
    },
  ];

  return (
    <section className={styles.chapterOverview}>
      <div className="container">
        <h2 className="text--center margin-bottom--lg">📖 What You'll Learn</h2>
        <div className="row">
          {chapters.map((part, idx) => (
            <div key={idx} className="col col--4 margin-bottom--lg">
              <div className={styles.chapterCard}>
                <h3>{part.title}</h3>
                <ul>
                  {part.items.map((item, i) => (
                    <li key={i}>{item}</li>
                  ))}
                </ul>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function Stats() {
  const chaptersCount = useCountUp(11);
  const wordsCount = useCountUp(47);
  const examplesCount = useCountUp(20);
  const quizzesCount = useCountUp(60);

  return (
    <section className={styles.stats}>
      <div className="container">
        <div className="row">
          <div className="col col--3 text--center" ref={chaptersCount.ref}>
            <div className={styles.statValue}>{chaptersCount.count}</div>
            <div className={styles.statLabel}>Chapters</div>
          </div>
          <div className="col col--3 text--center" ref={wordsCount.ref}>
            <div className={styles.statValue}>{wordsCount.count}K+</div>
            <div className={styles.statLabel}>Words</div>
          </div>
          <div className="col col--3 text--center" ref={examplesCount.ref}>
            <div className={styles.statValue}>{examplesCount.count}+</div>
            <div className={styles.statLabel}>Code Examples</div>
          </div>
          <div className="col col--3 text--center" ref={quizzesCount.ref}>
            <div className={styles.statValue}>{quizzesCount.count}+</div>
            <div className={styles.statLabel}>Quizzes</div>
          </div>
        </div>
      </div>
    </section>
  );
}

function Testimonials() {
  const testimonials = [
    {
      name: 'Sarah Chen',
      role: 'Robotics Engineer at Boston Dynamics',
      text: 'This textbook bridges theory and practice perfectly. The ROS 2 chapters alone are worth it!',
      avatar: '👩‍💻',
    },
    {
      name: 'Dr. James Wilson',
      role: 'AI Research Scientist',
      text: 'The VLA models chapter is cutting-edge. Best resource for Physical AI I\'ve found.',
      avatar: '👨‍🔬',
    },
    {
      name: 'Maria Rodriguez',
      role: 'Robotics Student',
      text: 'Interactive quizzes and code examples made learning so much easier. Highly recommend!',
      avatar: '👩‍🎓',
    },
  ];

  return (
    <section className={styles.testimonials}>
      <div className="container">
        <h2 className="text--center margin-bottom--lg">💬 What Students Say</h2>
        <div className="row">
          {testimonials.map((testimonial, idx) => (
            <div key={idx} className="col col--4">
              <div className={styles.testimonialCard}>
                <div className={styles.testimonialAvatar}>{testimonial.avatar}</div>
                <p className={styles.testimonialText}>"{testimonial.text}"</p>
                <div className={styles.testimonialAuthor}>
                  <strong>{testimonial.name}</strong>
                  <div className={styles.testimonialRole}>{testimonial.role}</div>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function Newsletter() {
  const [email, setEmail] = useState('');
  const [status, setStatus] = useState<'idle' | 'success'>('idle');

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    // In production, integrate with email service
    setStatus('success');
    setEmail('');
    setTimeout(() => setStatus('idle'), 3000);
  };

  return (
    <section className={styles.newsletter}>
      <div className="container">
        <div className="row">
          <div className="col col--8 col--offset-2 text--center">
            <h2>📬 Stay Updated</h2>
            <p className="margin-bottom--lg">
              Get notified about new chapters, updates, and robotics resources.
            </p>
            <form onSubmit={handleSubmit} className={styles.newsletterForm}>
              <input
                type="email"
                placeholder="Enter your email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                required
                className={styles.newsletterInput}
              />
              <button type="submit" className={styles.newsletterButton}>
                Subscribe
              </button>
            </form>
            {status === 'success' && (
              <div className={styles.successMessage}>
                ✅ Thanks for subscribing!
              </div>
            )}
          </div>
        </div>
      </div>
    </section>
  );
}


function CallToAction() {
  return (
    <section className={styles.cta}>
      <div className="container text--center">
        <h2>Ready to Master Physical AI?</h2>
        <p className="margin-bottom--lg">
          Start your journey into embodied intelligence and humanoid robotics today.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Get Started Now →
          </Link>
        </div>

        <div style={{ marginTop: '3rem', paddingTop: '2rem', borderTop: '1px solid var(--ifm-color-emphasis-300)' }}>
          <p style={{ fontSize: '1.1rem', fontWeight: 600, marginBottom: '1rem', color: 'var(--ifm-color-emphasis-800)' }}>
            Connect with the Author
          </p>
          <div style={{ display: 'flex', gap: '1rem', justifyContent: 'center', alignItems: 'center' }}>
            <a
              href="https://github.com/AbdulRehman2106"
              target="_blank"
              rel="noopener noreferrer"
              style={{
                display: 'inline-flex',
                alignItems: 'center',
                gap: '0.5rem',
                padding: '0.75rem 1.5rem',
                background: 'linear-gradient(135deg, #333 0%, #000 100%)',
                color: 'white',
                borderRadius: '8px',
                textDecoration: 'none',
                fontWeight: 600,
                transition: 'all 0.3s ease',
                boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
              }}
              onMouseOver={(e) => {
                e.currentTarget.style.transform = 'translateY(-2px)';
                e.currentTarget.style.boxShadow = '0 6px 20px rgba(0, 0, 0, 0.25)';
              }}
              onMouseOut={(e) => {
                e.currentTarget.style.transform = 'translateY(0)';
                e.currentTarget.style.boxShadow = '0 4px 12px rgba(0, 0, 0, 0.15)';
              }}
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
                <path d="M12 0c-6.626 0-12 5.373-12 12 0 5.302 3.438 9.8 8.207 11.387.599.111.793-.261.793-.577v-2.234c-3.338.726-4.033-1.416-4.033-1.416-.546-1.387-1.333-1.756-1.333-1.756-1.089-.745.083-.729.083-.729 1.205.084 1.839 1.237 1.839 1.237 1.07 1.834 2.807 1.304 3.492.997.107-.775.418-1.305.762-1.604-2.665-.305-5.467-1.334-5.467-5.931 0-1.311.469-2.381 1.236-3.221-.124-.303-.535-1.524.117-3.176 0 0 1.008-.322 3.301 1.23.957-.266 1.983-.399 3.003-.404 1.02.005 2.047.138 3.006.404 2.291-1.552 3.297-1.23 3.297-1.23.653 1.653.242 2.874.118 3.176.77.84 1.235 1.911 1.235 3.221 0 4.609-2.807 5.624-5.479 5.921.43.372.823 1.102.823 2.222v3.293c0 .319.192.694.801.576 4.765-1.589 8.199-6.086 8.199-11.386 0-6.627-5.373-12-12-12z"/>
              </svg>
              GitHub
            </a>
            <a
              href="https://www.linkedin.com/in/abdul-rehman-2213012b9/"
              target="_blank"
              rel="noopener noreferrer"
              style={{
                display: 'inline-flex',
                alignItems: 'center',
                gap: '0.5rem',
                padding: '0.75rem 1.5rem',
                background: 'linear-gradient(135deg, #0077b5 0%, #005582 100%)',
                color: 'white',
                borderRadius: '8px',
                textDecoration: 'none',
                fontWeight: 600,
                transition: 'all 0.3s ease',
                boxShadow: '0 4px 12px rgba(0, 119, 181, 0.3)',
              }}
              onMouseOver={(e) => {
                e.currentTarget.style.transform = 'translateY(-2px)';
                e.currentTarget.style.boxShadow = '0 6px 20px rgba(0, 119, 181, 0.5)';
              }}
              onMouseOut={(e) => {
                e.currentTarget.style.transform = 'translateY(0)';
                e.currentTarget.style.boxShadow = '0 4px 12px rgba(0, 119, 181, 0.3)';
              }}
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
                <path d="M20.447 20.452h-3.554v-5.569c0-1.328-.027-3.037-1.852-3.037-1.853 0-2.136 1.445-2.136 2.939v5.667H9.351V9h3.414v1.561h.046c.477-.9 1.637-1.85 3.37-1.85 3.601 0 4.267 2.37 4.267 5.455v6.286zM5.337 7.433c-1.144 0-2.063-.926-2.063-2.065 0-1.138.92-2.063 2.063-2.063 1.14 0 2.064.925 2.064 2.063 0 1.139-.925 2.065-2.064 2.065zm1.782 13.019H3.555V9h3.564v11.452zM22.225 0H1.771C.792 0 0 .774 0 1.729v20.542C0 23.227.792 24 1.771 24h20.451C23.2 24 24 23.227 24 22.271V1.729C24 .774 23.2 0 22.222 0h.003z"/>
              </svg>
              LinkedIn
            </a>
          </div>
          <p style={{ marginTop: '1rem', fontSize: '0.9rem', color: 'var(--ifm-color-emphasis-600)' }}>
            Created by <strong>Abdul Rehman</strong>
          </p>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="A comprehensive guide to Physical AI, ROS 2, simulation, VLA models, and humanoid robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <Stats />
        <ChapterOverview />
        <Testimonials />
        <Newsletter />
        <CallToAction />
      </main>
      <ChatBot />
    </Layout>
  );
}
